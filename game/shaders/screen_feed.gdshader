shader_type spatial;
// A camera feed on a cabin screen (ScreenCamera): the SubViewport's image is already tone-mapped for the display, so it goes
// through the display-referred path like every UI emitter (vision.gdshaderinc: past the eye's rod step and the display tone
// step unchanged, as display_colour.gdshader does for a flat colour).
render_mode unshaded, shadows_disabled, fog_disabled;

#include "res://shaders/vision.gdshaderinc"

uniform sampler2D feed : source_color, filter_linear, repeat_disable;
uniform float crop_y = 1.0;  // visible share of the feed's height (the feed is less wide than the screen: the screen shows its middle part)
uniform float live = 1.0;    // 0: no camera (dark screen)

void fragment() {
	vec2 uv = vec2(UV.x, 0.5 + (UV.y - 0.5) * crop_y);
	vec3 c = texture(feed, uv).rgb * live;
	ALBEDO = vision_display_referred(c);
}
