shader_type spatial;
// The Earth's ellipsoid mesh in scaled space without clouds (earth_surface.gdshaderinc); earth_clouds.gdshader with them.
render_mode unshaded, cull_back;

#include "res://shaders/earth_surface.gdshaderinc"
