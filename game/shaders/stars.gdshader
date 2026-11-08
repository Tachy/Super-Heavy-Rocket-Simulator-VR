shader_type spatial;
// Star points: vertex colour = B-V colour, CUSTOM0.x = the star's illuminance above the atmosphere (lux, StarField).
// Above the atmosphere a star renders at COLOR x clamp(E / reference_illuminance_lux, 0.04, 1) (the look before the
// atmosphere); through it the star is attenuated by the view ray's transmittance and hidden when its illuminance at
// the eye falls below the naked-eye threshold over the background (star_visibility.gdshaderinc): the larger of the sky
// behind the star and the luminance the eye is adapted to (atmosphere_adaptation_cd_m2, final review I1) - stars fade
// out in the day and twilight sky, stay hidden beside a sunlit Earth or with the Sun on the eye in space, and appear
// above the atmosphere once the eye has dark-adapted (night side, Earth's shadow). Evaluated per star (vertex).
render_mode unshaded, cull_disabled, depth_draw_never, blend_add;

#include "res://shaders/atmosphere.gdshaderinc"
#include "res://shaders/star_visibility.gdshaderinc"
#include "res://shaders/vision.gdshaderinc"
#include "res://shaders/refraction.gdshaderinc"

uniform float point_scale = 2.4;
// Illuminance rendered at full brightness (StarField.REFERENCE_MAGNITUDE).
uniform float reference_illuminance_lux = 1.0;

// [S: Wandell 1995 / Westheimer 1986, the dark-adapted eye's PSF core is ~1 arcmin FWHM] angular FWHM of the eye's core.
const float EYE_PSF_FWHM_RAD = 2.909e-4;
// [E] Smallest Gaussian sigma (pixels): a pixel-sized reconstruction filter, flux sampled at pixel centres stays within
// a few percent when the star moves by a sub-pixel amount.
const float PIXEL_SIGMA_MIN = 0.62;
// [E] sigma of a bright star as a fraction of the former square's side (size 5 px -> sigma 1.5 px).
const float GLARE_SIGMA_PER_SIZE = 0.30;
const float FWHM_PER_SIGMA = 2.35482;
const float KERNEL_SIGMAS = 3.5;  // sprite half width in sigmas

varying vec3 star_light;
varying float star_sigma_px;
varying vec2 star_centre_px;  // exact star centre in framebuffer pixels (POINT_COORD is invalid in the D3D12 multiview pipeline; Godot clip y points down, like FRAGCOORD.y)
varying float star_flux;  // star_light x former square area, integrated over the footprint

void vertex() {
	// The star at its refracted direction; the transmittance below follows the apparent ray.
	vec3 world = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
	VERTEX = (inverse(MODEL_MATRIX) * vec4(refracted_world_point(world, CAMERA_POSITION_WORLD), 1.0)).xyz;
	float illuminance = CUSTOM0.x;
	float relative = min(illuminance / reference_illuminance_lux, 1.0);
	vec3 attenuation = vec3(1.0);
	if (atmosphere_enabled) {
		vec3 direction_world = normalize((MODEL_MATRIX * vec4(VERTEX, 0.0)).xyz);
		vec3 transmittance;
		vec3 view_local = atmosphere_world_to_earth * direction_world;
		vec3 sky = atm_sky(atm_camera_local(), view_local, transmittance);
		if (atm_moonlit()) {
			sky += atm_moon_sky(atm_camera_local(), view_local);  // the moonlit sky behind the star
		}
		// Schaefer's B is the luminance the eye is adapted to; in a non-uniform field the adaptation luminance dominates
		// over a dark patch of sky (Ferwerda et al. 1996, threshold-versus-intensity of the adapted eye).
		float background_cd_m2 = max(dot(sky, LUMINANCE_WEIGHTS) / atmosphere_scene_per_cd_m2,
				atmosphere_adaptation_cd_m2);
		float at_eye_lux = illuminance * dot(transmittance, LUMINANCE_WEIGHTS);
		attenuation = transmittance * star_visibility(at_eye_lux, background_cd_m2);
	}
	star_light = COLOR.rgb * clamp(relative, 0.04, 1.0) * attenuation;
	float fade = dot(attenuation, LUMINANCE_WEIGHTS);
	float old_size_px = clamp(point_scale * sqrt(relative * fade) * 2.0, 1.0, 5.0);
	float px_per_rad = abs(PROJECTION_MATRIX[1][1]) * VIEWPORT_SIZE.y * 0.5;
	float psf_sigma_px = EYE_PSF_FWHM_RAD / FWHM_PER_SIGMA * px_per_rad;
	star_sigma_px = max(max(psf_sigma_px, PIXEL_SIGMA_MIN), GLARE_SIGMA_PER_SIZE * old_size_px);
	star_flux = old_size_px * old_size_px;
	vec4 clip = PROJECTION_MATRIX * (MODELVIEW_MATRIX * vec4(VERTEX, 1.0));
	star_centre_px = vec2(0.5 + 0.5 * clip.x / clip.w, 0.5 + 0.5 * clip.y / clip.w) * VIEWPORT_SIZE;
	POINT_SIZE = ceil(2.0 * KERNEL_SIGMAS * star_sigma_px) + 1.0;
}

void fragment() {
	// Display-referred brightness: bypasses night vision's rod step (vision.gdshaderinc).
	vec2 offset_px = FRAGCOORD.xy - star_centre_px;  // pixel centre minus the exact star centre
	float r2 = dot(offset_px, offset_px);
	float sigma2 = star_sigma_px * star_sigma_px;
	float profile = star_flux * exp(-0.5 * r2 / sigma2) / (6.2831853 * sigma2);
	ALBEDO = vision_display_referred(star_light) * profile;
}
