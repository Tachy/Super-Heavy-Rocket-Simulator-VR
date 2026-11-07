// Display-referred emitters under the eye model (game/world/eye_adaptation.gd, vision_effect.gd). The post step maps
// every pixel linearly: rgb' = C rgb + rod_scale x (w . rgb) x tint, C the cones' map (EyeAdaptation.cone_map: the white
// balance to the Sun's white, then the chroma, Hunt effect). An emitter whose on-screen colour is a display decision
// (the plume, the stars' own brightness, orbit lines, labels, the VR panel) writes the inverse of that linear map, so it
// leaves the step as drawn - also under alpha or additive blending (both linear). Sherman-Morrison:
//     q = C^-1 c,   c' = q - u (w . q),   u = rod_scale C^-1 tint / (1 + rod_scale w . C^-1 tint),
// vision_inverse_tint = u, vision_rod_weights = w, vision_cone_inverse = C^-1 (Atmosphere._set_vision); u = 0 and C = I
// with the adaptation off: the identity. The brightness factor (Stevens effect) is on the scene's pre-exposure
// (atmosphere_exposure_gain), which these emitters do not take: they do not dim.

global uniform vec3 vision_rod_weights;
global uniform vec3 vision_inverse_tint;
global uniform mat3 vision_cone_inverse;

vec3 vision_display_referred(vec3 c) {
	vec3 q = vision_cone_inverse * c;
	return q - vision_inverse_tint * dot(vision_rod_weights, q);
}
