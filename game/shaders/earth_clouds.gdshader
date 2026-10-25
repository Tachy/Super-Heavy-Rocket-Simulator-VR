shader_type spatial;
// The Earth's ellipsoid mesh in scaled space with the far-field clouds (earth_surface.gdshaderinc with EARTH_CLOUDS,
// cloud_far.gdshaderinc); World draws it while CloudLayer.far_enabled, earth.gdshader otherwise.
render_mode unshaded, cull_back;

#define EARTH_CLOUDS
#include "res://shaders/earth_surface.gdshaderinc"
