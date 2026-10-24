// Geomorphing of terrain quadtree nodes (TerrainQuadtree), shared by the Earth and Moon terrain shaders.
// A node's vertices slide towards the parent level's surface (CUSTOM0.xyz) and their normals towards the parent level's
// normals (CUSTOM3.xyz) as the camera moves away: morph is 0 closer than morph_start_m and 1 beyond morph_end_m. The
// quadtree sets both per node (TerrainQuadtree.morph_range_m) so that a node is unmorphed whenever its children appear
// or vanish and a child is fully morphed (= its parent's surface) at that moment: splits and merges never pop.
// A node drawn for the first time also starts fully morphed through morph_floor (1 -> 0 over TerrainQuadtree.MORPH_IN_S),
// which covers children whose upload came late (the camera already inside their morph range).

// The node centre in body-local render metres as float32 hi + lo parts (TerrainQuadtree.split_float32; vertices are
// stored relative to it); set once per node.
instance uniform vec3 vertex_offset = vec3(0.0);
instance uniform vec3 vertex_offset_lo = vec3(0.0);
instance uniform float morph_start_m = 1.0e30;
instance uniform float morph_end_m = 2.0e30;
instance uniform float morph_floor = 0.0;

// Camera relative to the node centre (node-local render axes, metres) from the camera's body-local position in hi + lo
// parts (a global per body, written once per frame): the hi difference of nearby points is exact in float32, so the
// result keeps float32 precision relative to the camera-node distance.
vec3 terrain_camera_local(vec3 camera_hi, vec3 camera_lo) {
	return (camera_hi - vertex_offset) + (camera_lo - vertex_offset_lo);
}

// Morph factor of a vertex at node-local position `vertex` for the camera at node-local camera_local.
float terrain_morph_factor(vec3 vertex, vec3 camera_local) {
	float by_distance = clamp((length(vertex - camera_local) - morph_start_m) / (morph_end_m - morph_start_m), 0.0, 1.0);
	return max(by_distance, morph_floor);
}
