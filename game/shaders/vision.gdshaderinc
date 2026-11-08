// Display-referred emitters under the eye model (game/world/eye_adaptation.gd, vision_effect.gd). The post step maps
// every pixel linearly: rgb' = C rgb + rod_scale x (w . rgb) x tint, C the cones' map (EyeAdaptation.cone_map: the white
// balance to the Sun's white, then the chroma, Hunt effect, its part <= 1). An emitter whose on-screen colour is a display decision
// (the plume, the stars' own brightness, orbit lines, labels, the VR panel) writes the inverse of that linear map, so it
// leaves the step as drawn - also under alpha or additive blending (both linear). Sherman-Morrison:
//     q = C^-1 c,   c' = q - u (w . q),   u = rod_scale C^-1 tint / (1 + rod_scale w . C^-1 tint),
// vision_inverse_tint = u, vision_rod_weights = w, vision_cone_inverse = C^-1 (Atmosphere._set_vision); u = 0 and C = I
// with the adaptation off: the identity. The brightness factor (Stevens effect) is on the scene's pre-exposure
// (atmosphere_exposure_gain), which these emitters do not take: they do not dim.
// The display tone step after it (DisplayToneEffect, look fixes T5: a chroma-preserving curve on AgX's grey curve, the
// daylight Hunt factor in OkLab) is not linear: the emitter first takes its inverse (DisplayTone.tone_inverse /
// hunt_inverse) - AgX's colour of c scaled to the scene luminance the curve shows at its luminance, then the OkLab
// chroma / vision_hunt - so it is shown as AgX alone showed it. Within ~1 % for an emitter alone on its pixel (on
// black, opaque; below scene 2 and the gamut knee; the Hunt inverse keeps the OkLab lightness, the curve reads the
// luminance), approximate where it blends with the scene. vision_tone = the allenwp
// curve (contrast, toe_a, slope, w), vision_tone_white AgX's white; contrast 0 and vision_hunt 1 while the step is off.

global uniform vec3 vision_rod_weights;
global uniform vec3 vision_inverse_tint;
global uniform mat3 vision_cone_inverse;
global uniform float vision_hunt;
global uniform vec4 vision_tone;
global uniform float vision_tone_white;
// The fixed display curve of the tone step's step 1 (DisplayTone.display_curve, look fixes T13): slope, knee, 1 = on
// (0: AgX's grey curve, vision_curve_inverse).
global uniform vec3 vision_display_curve;

float vision_curve(float x) {
	x = max(x, 0.0);
	if (x < 0.18) {
		float t = pow(x, vision_tone.x);
		return t / (t + vision_tone.y);
	}
	float s = x - 0.18;
	float slope_s = vision_tone.z * s;
	return min(slope_s * (1.0 + s / vision_tone.w) / (1.0 + slope_s / 0.82) + 0.18, 1.0);
}

float vision_curve_inverse(float y) {
	y = clamp(y, 0.0, 0.999999);
	if (y < 0.18) {
		return pow(vision_tone.y * y / (1.0 - y), 1.0 / vision_tone.x);
	}
	float d = y - 0.18;
	float a = vision_tone.z / vision_tone.w;
	float b = vision_tone.z * (1.0 - d / 0.82);
	return 2.0 * d / (b + sqrt(max(b * b + 4.0 * a * d, 0.0))) + 0.18;
}

// [S: Godot 4.7 tonemap.glsl tonemap_agx] AgX's colour of c, then the scene value of that chromaticity at the
// luminance the curve shows at its luminance (DisplayTone.tone_inverse).
vec3 vision_tone_inverse(vec3 c) {
	c = max(c, vec3(0.0));
	const vec3 LUMINANCE = vec3(0.2126, 0.7152, 0.0722);
	if (vision_tone.x == 0.0 || dot(LUMINANCE, c) >= vision_tone_white) {
		return c;
	}
	const mat3 INSET = mat3(vec3(0.544814746488245, 0.140416948464053, 0.0888104196149096),
			vec3(0.373787398372697, 0.754137554567394, 0.178871756420858),
			vec3(0.0813978551390581, 0.105445496968552, 0.732317823964232));
	const mat3 OUTSET = mat3(vec3(1.96488741169489, -0.299313364904742, -0.164352742528393),
			vec3(-0.855988495690215, 1.32639796461980, -0.238183969428088),
			vec3(-0.108898916004672, -0.0270845997150571, 1.40253671195648));
	vec3 u = INSET * c;
	vec3 d = clamp(OUTSET * vec3(vision_curve(u.x), vision_curve(u.y), vision_curve(u.z)), vec3(0.0), vec3(1.0));
	float y = dot(LUMINANCE, d);
	if (y <= 0.0) {
		return vec3(0.0);
	}
	float scene_y = vision_curve_inverse(y);
	if (vision_display_curve.z != 0.0) {
		float knee = vision_display_curve.y;
		float yc = min(y, 0.999999);
		scene_y = (yc <= knee ? yc : knee - (1.0 - knee) * log((1.0 - yc) / (1.0 - knee))) / vision_display_curve.x;
	}
	return d * (scene_y / y);
}

// [S: B. Ottosson 2020, "A perceptual color space for image processing"] linear sRGB <-> OkLab (DisplayTone).
vec3 vision_hunt_inverse(vec3 c) {
	if (vision_hunt == 1.0) {
		return c;
	}
	const mat3 M1 = mat3(vec3(0.4122214708, 0.2119034982, 0.0883024619), vec3(0.5363325363, 0.6806995451, 0.2817188376),
			vec3(0.0514459929, 0.1073969566, 0.6299787005));
	const mat3 M2 = mat3(vec3(0.2104542553, 1.9779984951, 0.0259040371), vec3(0.7936177850, -2.4285922050, 0.7827717662),
			vec3(-0.0040720468, 0.4505937099, -0.8086757660));
	vec3 lms = M1 * c;
	vec3 lab = M2 * (sign(lms) * pow(abs(lms), vec3(1.0 / 3.0)));
	lab.yz /= vision_hunt;
	lms = inverse(M2) * lab;
	return inverse(M1) * (lms * lms * lms);
}

vec3 vision_display_referred(vec3 c) {
	vec3 q = vision_cone_inverse * vision_hunt_inverse(vision_tone_inverse(c));
	return q - vision_inverse_tint * dot(vision_rod_weights, q);
}
