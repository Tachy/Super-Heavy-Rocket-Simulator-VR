// Display-referred emitters under the eye model (game/world/eye_adaptation.gd, vision_effect.gd). The post step maps
// every pixel linearly: rgb' = C rgb + rod_scale x (w . rgb) x tint, C rgb = Y + chroma (rgb - Y) (the cones' chroma,
// Hunt effect; Y the BT.709 luminance). An emitter whose on-screen colour is a display decision (the plume, the stars'
// own brightness, orbit lines, labels, the VR panel) writes the inverse of that linear map, so it leaves the step as
// drawn - also under alpha or additive blending (both linear). Sherman-Morrison:
//     q = C^-1 c = (c - (1 - chroma) Y) / chroma,   c' = q - u (w . q),   u = rod_scale C^-1 tint / (1 + rod_scale w . C^-1 tint),
// vision_inverse_tint = u, vision_rod_weights = w, vision_chroma = chroma (Atmosphere._set_vision); u = 0 and chroma 1
// in daylight or with the adaptation off: the identity. The brightness factor (Stevens effect) is on the scene's
// pre-exposure (atmosphere_exposure_gain), which these emitters do not take: they do not dim.

global uniform vec3 vision_rod_weights;
global uniform vec3 vision_inverse_tint;
global uniform float vision_chroma;

vec3 vision_display_referred(vec3 c) {
	float y = dot(vec3(0.2126, 0.7152, 0.0722), c);  // [S: ITU-R BT.709]
	vec3 q = (c - vec3((1.0 - vision_chroma) * y)) / max(vision_chroma, 1.0e-3);
	return q - vision_inverse_tint * dot(vision_rod_weights, q);
}
