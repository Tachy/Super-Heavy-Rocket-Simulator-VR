// Display-referred emitters under night vision (game/world/eye_adaptation.gd, vision_effect.gd). The post step adds
// the rods' image to every pixel: rgb + rod_scale x (w . rgb) x tint. An emitter whose on-screen colour is a display
// decision (the plume, the city lights, the stars' own brightness, orbit lines, labels, the VR panel) writes the
// inverse of that linear map, so it leaves the step as drawn - also under alpha or additive blending (both linear):
//     c' = c - tint x rod_scale / (1 + rod_scale w . tint) x (w . c),
// vision_inverse_tint = tint x rod_scale / (1 + rod_scale w . tint), vision_rod_weights = w (Atmosphere.update); zero
// in daylight or with the adaptation off: the identity. (The step's max(w . rgb, 0) keeps the sign of w . c', so the
// inverse is exact wherever the pixel's scotopic part is not negative.)

global uniform vec3 vision_rod_weights;
global uniform vec3 vision_inverse_tint;

vec3 vision_display_referred(vec3 c) {
	return c - vision_inverse_tint * dot(vision_rod_weights, c);
}
