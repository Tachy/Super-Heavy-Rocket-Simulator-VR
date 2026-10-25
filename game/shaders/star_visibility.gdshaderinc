// Naked-eye visibility of a point source (a star) over a uniform background (the sky behind it), M8 atmosphere.
//
// Threshold illuminance of a point source over a background of brightness B (Schaefer's form of Hecht's 1947 data, as
// used by Garstang 1986 for the night sky):
//     I_th = c1 (1 + sqrt(c2 B))^2,   I_th in foot-candles, B in nanolamberts,
// with c1 = 10^-9.80, c2 = 10^-1.90 (night vision, rods) and c1 = 10^-8.35, c2 = 10^-5.90 (day vision, cones)
// [S: B. E. Schaefer 1990, "Telescopic limiting magnitudes", PASP 102, 212, eq. 2 and table; the same constants in his
// program "To the Visual Limits", Sky & Telescope 95 (5), 57, 1998]. Schaefer switches branches at B = 1500 nL; the
// smaller of the two thresholds is taken here [E] (the branches cross near 1700 nL, 7 % apart at 1500 nL: continuous,
// no visible step). Checks: B = 0 gives a limiting magnitude 7.9, a dark sky (54 nL, 22 mag/arcsec^2) 6.6, the noon sky
// (~5000 cd/m^2) about -4 (Venus is a daytime object, magnitude-0 stars are not).
// Not modelled (an average observer): age, experience, seeing, the Moon's or light pollution's sky. The background
// passed in (stars.gdshader) is the larger of the atmosphere model's sunlit sky behind the star and the luminance the
// eye is adapted to (EyeAdaptation, paced with its adaptation times): the eye adapted to a sunlit Earth or the Sun in
// view does not see faint stars against black space.

const float SCHAEFER_NIGHT_C1_FC = 1.5849e-10;   // 10^-9.80 foot-candles
const float SCHAEFER_NIGHT_C2_PER_NL = 1.2589e-2;  // 10^-1.90 per nanolambert
const float SCHAEFER_DAY_C1_FC = 4.4668e-9;      // 10^-8.35 foot-candles
const float SCHAEFER_DAY_C2_PER_NL = 1.2589e-6;    // 10^-5.90 per nanolambert
// [S: definitions] 1 lambert = 1e4 / pi cd/m^2, so 1 cd/m^2 = pi x 1e5 nL; 1 foot-candle = 1 lm/ft^2 = 10.7639 lux.
const float NANOLAMBERTS_PER_CD_M2 = 314159.27;
const float LUX_PER_FOOT_CANDLE = 10.763910;
// [E] Half width (magnitudes) of the smooth step around the threshold: detection probability rises over about a
// magnitude (Blackwell 1946's frequency-of-seeing curves span ~0.2-0.3 in log contrast).
const float STAR_VISIBILITY_HALF_WIDTH_MAG = 0.5;
// [S: ITU-R BT.709 / sRGB] luminance weights of the linear RGB channels.
const vec3 LUMINANCE_WEIGHTS = vec3(0.2126, 0.7152, 0.0722);

// Threshold illuminance (lux) of a point source over a background luminance (cd/m^2).
float star_threshold_lux(float background_luminance_cd_m2) {
	float b_nl = max(background_luminance_cd_m2, 0.0) * NANOLAMBERTS_PER_CD_M2;
	float night = 1.0 + sqrt(SCHAEFER_NIGHT_C2_PER_NL * b_nl);
	float day = 1.0 + sqrt(SCHAEFER_DAY_C2_PER_NL * b_nl);
	return min(SCHAEFER_NIGHT_C1_FC * night * night, SCHAEFER_DAY_C1_FC * day * day) * LUX_PER_FOOT_CANDLE;
}

// Visibility 0..1 of a star of illuminance star_illuminance_lux (at the eye) over the background: 0.5 at the
// threshold, smooth over +-STAR_VISIBILITY_HALF_WIDTH_MAG magnitudes around it.
float star_visibility(float star_illuminance_lux, float background_luminance_cd_m2) {
	float ratio = max(star_illuminance_lux, 1.0e-30) / star_threshold_lux(background_luminance_cd_m2);
	float margin_mag = 2.5 * log(ratio) / log(10.0);  // > 0: brighter than the threshold
	return smoothstep(-STAR_VISIBILITY_HALF_WIDTH_MAG, STAR_VISIBILITY_HALF_WIDTH_MAG, margin_mag);
}
