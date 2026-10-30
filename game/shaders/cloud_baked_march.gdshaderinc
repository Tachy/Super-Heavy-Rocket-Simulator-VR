// The march through the sets and the ray's shading (cloud_pass.gdshaderinc's main calls cloud_baked_shade;
// cloud_baked.gdshaderinc).

#ifdef CLOUD_BAKED_MARCH
// The march of one set: dir (Earth-local unit) from eye over [d0, d1] of the band. Out: radiance (before the air in
// front), the background's transmittance and the alpha-weighted distance.
void cloud_baked_march(int si, CloudRay ray, vec3 dir,
		vec3 eye, float d0, float d1, out vec3 radiance, out float transmittance_bg, out float weighted) {
	radiance = vec3(0.0);
	transmittance_bg = 1.0;
	weighted = 0.0;
	CloudBakedRay g = cloud_baked_ray(si, eye, dir);
	vec3 camera = atmosphere_camera_hi + atmosphere_camera_lo;
	float k_sc = cloud_transport_scale();
	bool table = cloud_scattering_model == 1;
	float w_light = table ? cloud_phase.w : k_sc;
	vec3 sun_dir = normalize(atmosphere_sun_direction);
	bool sun_ray = dot(dir, sun_dir) > cos(CLOUD_SUN_RAY_ANGLE);
	vec3 diff = sun_dir - dir;  // Sun propagation (-sun_dir) . view propagation (-dir): the scattering angle
	float one_minus_cos = 0.5 * dot(diff, diff);
	float phase = cloud_phase_function(1.0 - one_minus_cos, one_minus_cos);
	float transmittance_sc = 1.0;
	float transmittance_l = 1.0;
	vec3 sun_irradiance = vec3(-1.0);  // set at the first cloud sample (one air lookup per ray)
	vec3 sky_irradiance = vec3(0.0);
	int levels = int(baked.sets[si].axis_y.w);
	float c0 = baked.sets[si].axis_x.w;
	float n = baked.sets[si].up0.w;
	float pixel_angle = cloud_view.params.w;
	int k_min = int(baked.sets[si].misc.z);
	float t = d0;
	float t_clear = d0;     // the last clear sample (an entry is bisected from it)
	bool inside = false;    // the last sample had extinction: the next steps are in-cloud steps
	float beta_last = 0.0;  // in cloud: the extinction and light at t
	vec4 light_last = vec4(0.0);
	for (int i = 0; i < CLOUD_BAKED_MAX_STEPS && t < d1; i++) {
		float h = cloud_height_along(ray, t);
		// (a tolerance: at the band's crossings the float32 height lands either side - a ray starting on the shell
		// was sent to the shell's far crossing, skipping the band)
		if (h > baked.sets[si].band.y + CLOUD_BAKED_BAND_TOLERANCE_M
				|| h < baked.sets[si].band.x - CLOUD_BAKED_BAND_TOLERANCE_M) {
			// Outside the band (a ray grazing its bottom): on to where it enters again.
			vec2 x = cloud_sphere_crossing(ray, h > baked.sets[si].band.y ? baked.sets[si].band.y : baked.sets[si].band.x);
			float enter = x.x > t ? x.x : (x.y > t ? x.y : 1.0e30);
			if (enter >= d1) {
				break;
			}
			t = enter + 1.0;
			t_clear = t;
			inside = false;
			continue;
		}
		float gx = g.gx0 + t * g.gdx;
		float gy = g.gy0 + t * g.gdy;
		// The level rule (cloud_baked_lod: the clipmap's extent and the footprint, never a dropped level; blended toward
		// the next level over its last cloud_baked_band()) and the hand-over to the flat levels: the extinction
		// scaled by 1 - w; the flat levels' alone from w = 1 on (w grows along the ray: x, y and the footprint grow with t
		// from the anchor).
		CloudBakedPoint q = cloud_baked_point(si, gx, gy, h, pixel_angle * t, k_min);
		if (q.keep <= 0.0) {
			break;
		}
		int k = q.k;
		if (k >= levels) {
			break;  // beyond the outermost level: the flat levels (cloud_baked_set)
		}
		float c = c0 * exp2(float(k));
		int kq = k >> 2;
		int kr = k & 3;
		float dz = baked.sets[si].dz[kq][kr];
		float slope = cloud_height_slope(ray, t);
		float cell_step = min(c, dz / max(abs(slope), 1.0e-3));
		if (!inside) {
			// The block's occupancy: an empty block is skipped to its exit (x, y linear; the height shells exact).
			vec3 cg = vec3(gx / c + 0.5 * n, gy / c + 0.5 * n, (h - baked.sets[si].band.x) / dz);
			ivec3 b = ivec3(floor(cg / float(CLOUD_BAKED_BLOCK)));
			float occ = cloud_baked_o(si, ivec3(b.xy, int(baked.sets[si].oslab[kq][kr])
					+ clamp(b.z, 0, int(baked.sets[si].onz[kq][kr]) - 1)));
			if (occ < 0.5 && q.blend > 0.0) {
				// blending: the next level's block too (its cells may reach where this level's are clear)
				int k1 = k + 1;
				float c1 = 2.0 * c;
				float dz1 = baked.sets[si].dz[k1 >> 2][k1 & 3];
				ivec3 b1 = ivec3(floor(vec3(gx / c1 + 0.5 * n, gy / c1 + 0.5 * n, (h - baked.sets[si].band.x) / dz1)
						/ float(CLOUD_BAKED_BLOCK)));
				occ = cloud_baked_o(si, ivec3(b1.xy, int(baked.sets[si].oslab[k1 >> 2][k1 & 3])
						+ clamp(b1.z, 0, int(baked.sets[si].onz[k1 >> 2][k1 & 3]) - 1)));
			}
			if (occ < 0.5) {
				float bs = float(CLOUD_BAKED_BLOCK);
				float exit_m = 1.0e30;
				if (g.gdx != 0.0) {
					float edge = ((float(b.x) + (g.gdx > 0.0 ? 1.0 : 0.0)) * bs - 0.5 * n) * c;
					exit_m = min(exit_m, (edge - gx) / g.gdx);
				}
				if (g.gdy != 0.0) {
					float edge = ((float(b.y) + (g.gdy > 0.0 ? 1.0 : 0.0)) * bs - 0.5 * n) * c;
					exit_m = min(exit_m, (edge - gy) / g.gdy);
				}
				// The block's height exit by the ray's direction (cirrus check): rising, the top shell's forward crossing;
				// descending, the bottom's first crossing, else (it turns back up above the bottom) the top's. Taken by
				// the direction, never by "the first root ahead of t": the roots' float32 rounding (~1 m at 6 378 km
				// radii) against cloud_height_along's put a ray just under the top shell past both roots, so the
				// skip went to an x / y face kilometres away - straight through the sheet (pixel holes in cirrus
				// overhead, square rings along the blocks). A root just behind t means the shell is here (exit 0).
				float z_lo = baked.sets[si].band.x + float(b.z) * bs * dz;
				vec2 hi = cloud_sphere_crossing(ray, z_lo + bs * dz);
				float next_z = hi.y;
				if (slope < 0.0) {
					vec2 lo = cloud_sphere_crossing(ray, z_lo);
					if (lo.x <= lo.y && lo.x > t - CLOUD_BAKED_BAND_TOLERANCE_M) {
						next_z = lo.x;
					}
				}
				exit_m = min(exit_m, max(next_z, t) - t);
				t += max(exit_m, 0.0) + max(CLOUD_BAKED_EXIT_NUDGE * c, 1.0);
				t_clear = t;
				continue;
			}
			// An occupied block: a sample per cell far from any edge, nearer the edge coordinate's distance estimate (fix 1:
			// one sample per 100 m cell missed the rims' thin parts ray by ray - holes, speckles and stairs at near
			// outlines), never below the pixel's footprint.
			float gap_x;
			float beta = cloud_baked_beta_gap(si, q, eye + dir * t, pixel_angle * t, gap_x);
			if (beta <= 0.0) {
				t_clear = t;
				t += transmittance_sc > CLOUD_BAKED_FINE_TRANSMITTANCE
						? clamp(gap_x * CLOUD_BAKED_EDGE_STEP_M, max(CLOUD_BAKED_MIN_CLEAR_STEP_M, pixel_angle * t), cell_step)
						: cell_step;
				continue;
			}
			// The entry: bisected between the last clear sample and this one (the outline found, not sampled).
			float lo_t = t_clear;
			float hi_t = t;
			for (int j = 0; j < CLOUD_BAKED_BISECTIONS; j++) {
				float mid = 0.5 * (lo_t + hi_t);
				float hm = cloud_height_along(ray, mid);
				// (the sample's level and blend: the outline's position, not its level, is searched)
				CloudBakedPoint qm = q;
				qm.g = vec3(g.gx0 + mid * g.gdx, g.gy0 + mid * g.gdy, hm);
				qm.uvw = cloud_baked_uvw(si, k, qm.g.x, qm.g.y, hm);
				float bm = cloud_baked_beta(si, qm, eye + dir * mid, pixel_angle * mid);
				if (bm > 0.0) {
					hi_t = mid;
					beta = bm;
				} else {
					lo_t = mid;
				}
			}
			// The in-cloud steps start at the entry's cloudy end, its extinction and light known.
			t = hi_t;
			beta_last = beta;
			float h_e = cloud_height_along(ray, t);
			light_last = cloud_baked_b2(si, cloud_baked_point(si, g.gx0 + t * g.gdx, g.gy0 + t * g.gdy, h_e,
					pixel_angle * t, k_min));
			light_last *= light_last;  // the sqrt code
			inside = true;
			continue;
		}
		// In cloud: a step of at most the step weight (the old march's dense limit) and a quarter / half a cell
		// (CLOUD_BAKED_FINE_STEP_CELLS), integrated by the trapezoid over the extinction at its ends (a layer thinner than
		// the step still counts).
		float allowed = CLOUD_BAKED_STEP_WEIGHT / transmittance_sc;
		float dense = allowed < 1.0 ? -log(1.0 - allowed) / (k_sc * max(beta_last, 1.0e-7)) : 1.0e30;
		float step_m = clamp(dense, CLOUD_BAKED_MIN_STEP_M, (transmittance_sc > CLOUD_BAKED_FINE_TRANSMITTANCE
				? CLOUD_BAKED_FINE_STEP_CELLS : CLOUD_BAKED_COARSE_STEP_CELLS) * cell_step);
		step_m = min(step_m, d1 - t);
		float te = t + step_m;
		float he = cloud_height_along(ray, te);
		// (the step's start's level, blend and flat weight, as the volume's march took the start's level)
		CloudBakedPoint qe = q;
		qe.g = vec3(g.gx0 + te * g.gdx, g.gy0 + te * g.gdy, he);
		qe.uvw = cloud_baked_uvw(si, k, qe.g.x, qe.g.y, he);
		float beta_e = cloud_baked_beta(si, qe, eye + dir * te, pixel_angle * te);
		vec4 light_e = light_last;
		if (beta_e > 0.0) {
			light_e = cloud_baked_b2(si, qe);
			light_e *= light_e;  // the sqrt code
		}
		float beta = 0.5 * (beta_last + beta_e);
		vec4 light = (beta_last * light_last + beta_e * light_e) / max(beta_last + beta_e, 1.0e-12);
		float d = t + 0.5 * step_m;
		t = te;
		beta_last = beta_e;
		light_last = light_e;
		if (beta_e <= 0.0) {
			inside = false;
			t_clear = te;
		}
		if (sun_irradiance.x < 0.0) {
			vec3 p = camera + eye + dir * d;
			vec3 normal = normalize(p * ATM_STRETCH * ATM_STRETCH);
			sun_irradiance = ATM_PI * atm_sun_at(p) * atm_sunlit(p);
			float eclipse_x = clamp((d - CLOUD_ECLIPSE_NEAR_M) / (CLOUD_ECLIPSE_FAR_M - CLOUD_ECLIPSE_NEAR_M), 0.0,
					1.0);
			sky_irradiance = ATM_PI * atm_sky_light(p, normal)
					* mix(cloud_group_eclipse_near, cloud_group_eclipse_far, eclipse_x);
#ifdef CLOUD_DIAG_SUN_ONLY
			sky_irradiance = vec3(0.0);
#endif
#ifdef CLOUD_DIAG_SKY_ONLY
			sun_irradiance = vec3(0.0);
#endif
		}
		float tau = beta * step_m;
		float scaled_step = exp(-k_sc * tau);
		float light_step = exp(-w_light * tau);
		float true_step = exp(-tau);
		if (transmittance_sc >= CLOUD_MIN_SCALED_TRANSMITTANCE) {
			float j_sun = phase * light.x + CLOUD_BAKED_J_MAX * light.y;
			radiance += transmittance_sc * sky_irradiance * (CLOUD_BAKED_J_MAX * light.z) * (1.0 - scaled_step) / k_sc
					+ transmittance_l * sun_irradiance * j_sun * (1.0 - light_step) / w_light;
		}
		transmittance_sc *= scaled_step;
		transmittance_l *= light_step;
		float next_bg = transmittance_bg * true_step;
		weighted += d * (transmittance_bg - next_bg);
		transmittance_bg = next_bg;
		if (transmittance_sc < CLOUD_MIN_SCALED_TRANSMITTANCE
				&& !(sun_ray && transmittance_bg > CLOUD_SUN_MIN_TRANSMITTANCE)) {
			break;
		}
	}
}

// One set's clouds along the ray: the volume's march over its band segment [d0, d1] (volume: the segment exists; faded
// out toward the flat levels) and the flat levels behind it. Out: premultiplied colour with the air in front, the
// background's transmittance and the alpha-weighted cloud distance.
void cloud_baked_set(int si, CloudRay ray, vec3 dir, vec3 eye, float d0, float d1, bool volume, float scene_m,
		out vec3 color, out float transmittance, out float cloud_m, out float far_w) {
	color = vec3(0.0);
	transmittance = 1.0;
	cloud_m = 0.0;
	far_w = 0.0;
	float alpha = 0.0;
	if (volume && int(baked.sets[si].misc.z) < int(baked.sets[si].axis_y.w) && cloud_baked_takeover_weight(si) < 1.0) {
		vec3 radiance;
		float weighted;
		cloud_baked_march(si, ray, dir, eye, d0, d1, radiance, transmittance, weighted);
		alpha = 1.0 - transmittance;
		cloud_m = alpha > 1.0e-6 ? weighted / alpha : 0.0;
		if (alpha > 1.0e-6) {
			vec3 air_transmittance;
			vec3 air_light = atm_to_point_offset(eye + dir * cloud_m, air_transmittance);
			color = air_transmittance * radiance + alpha * air_light;
		}
	}
	// The flat levels from where their weight starts, inside the 20 km shell: behind the volume by the hand-over's
	// weight (render A), alone (render B), A and B cross-faded by the takeover's weight.
	float wt = cloud_baked_takeover_weight(si);
	if (baked.sets[si].outer.x < 0.5 || (transmittance < CLOUD_BAKED_FLAT_MIN_TRANSMITTANCE && wt <= 0.0)) {
		return;
	}
#ifdef CLOUD_BAKED_VOLUME_ONLY
	return;
#endif
	float pixel_angle = cloud_view.params.w;
	CloudBakedRay g = cloud_baked_ray(si, eye, dir);
	float t_from = cloud_baked_flat_start(si, g, pixel_angle);
	vec2 shell = cloud_sphere_crossing(ray, CLOUD_TOP_M);
	vec3 color_f = vec3(0.0);
	float alpha_f = 0.0;
	float dist_f = 0.0;
	// beyond the flat geometry's cap the far field's column takes over (its share far_w: cloud_baked_shade adds it)
#ifdef CLOUD_BAKED_FAR_FALLBACK
	far_w = cloud_baked_flat_far_weight(si, ray, dir, eye);
#endif
	if (far_w >= 1.0 || shell.x > shell.y || shell.y <= t_from || t_from >= scene_m
			|| !cloud_baked_flat_shade(si, ray, dir, eye, max(t_from, shell.x), scene_m, pixel_angle, color_f, alpha_f,
					dist_f)) {
		color_f = vec3(0.0);
		alpha_f = 0.0;
	}
	color_f *= 1.0 - far_w;
	alpha_f *= 1.0 - far_w;
	if (alpha_f > 0.0 && transmittance >= CLOUD_BAKED_FLAT_MIN_TRANSMITTANCE) {
		float w = cloud_baked_flat_weight_at(si, cloud_baked_lod(si, max(abs(g.gx0 + dist_f * g.gdx),
				abs(g.gy0 + dist_f * g.gdy)), pixel_angle * dist_f));
		float alpha_w = alpha_f * w;
		float alpha_all = alpha + transmittance * alpha_w;
		cloud_m = alpha_all > 1.0e-6 ? (alpha * cloud_m + transmittance * alpha_w * dist_f) / alpha_all : 0.0;
		color += transmittance * w * color_f;
		transmittance *= 1.0 - alpha_w;
	}
	if (wt > 0.0) {
		// Region by region (cloud_baked_takeover_local): each ground patch shows one render, its edge soft.
		vec2 x2 = cloud_sphere_crossing(ray, CLOUD_BAKED_FLAT_PATTERN_HEIGHT_M);
		float t_p = x2.x <= x2.y && x2.x > 0.0 ? x2.x : (x2.y > 0.0 ? x2.y : CLOUD_BAKED_FLAT_TAKEOVER_HEIGHTS_M.y);
		wt = cloud_baked_takeover_local(atmosphere_camera_hi + atmosphere_camera_lo + eye + dir * t_p, wt);
		float alpha_a = 1.0 - transmittance;
		float alpha_mix = mix(alpha_a, alpha_f, wt);
		cloud_m = alpha_mix > 1.0e-6 ? mix(alpha_a * cloud_m, alpha_f * dist_f, wt) / alpha_mix : 0.0;
		color = mix(color, color_f, wt);
		transmittance = 1.0 - alpha_mix;
	}
}

// One ray of the baked volume and its flat outer levels: dir (Earth-local unit) from eye up to scene_m. Out: the stored
// colour (cloud_stored_color: premultiplied colour with the air in front and the air shadow's loss, T in .a) and the
// alpha-weighted cloud distance.
void cloud_baked_shade(vec3 dir, vec3 eye, float scene_m, out vec4 color_t, out float cloud_m) {
#ifdef CLOUD_BAKED_DIAG_NO_SCENE
	scene_m = 1.0e30;  // (diagnostic: the scene's depth ignored)
#endif
	cloud_m = 0.0;
	color_t = vec4(0.0, 0.0, 0.0, 1.0);
	if (!cloud_enabled || !atmosphere_enabled) {
		return;
	}
	const int si = 0;
	CloudRay ray = cloud_ray_from(dir, eye);
#ifdef CLOUD_BAKED_FAR_REFERENCE
	{
		// (diagnostics: the scaled-space far field's cloud layer, cloud_far.gdshaderinc cloud_far_layer, at the ray's
		// ground point instead of the baked clouds - the flat levels' cover reference at any height)
		vec2 ground = cloud_sphere_crossing(ray, 0.0);
		if (ground.x <= ground.y && ground.x > 0.0) {
			vec4 layer = cloud_far_layer(atmosphere_camera_hi + atmosphere_camera_lo + eye + dir * ground.x, vec3(1.0));
			cloud_m = layer.a > 1.0e-6 ? ground.x : 0.0;
			color_t = cloud_stored_color(layer.rgb, 1.0 - layer.a);
		}
		return;
	}
#endif
#ifdef CLOUD_BAKED_FLAT_TAP_STATS
	{
		// (diagnostics: the flat cell's step 2 tap statistics at the ray's 5 km crossing, raw: rgb = cover, the share of
		// it from taps of opacity < 0.1, < 0.3; a = 1 - the mean opacity)
		vec2 x = cloud_sphere_crossing(ray, 5000.0);
		vec4 f;
		vec2 t_col;
		if (x.x <= x.y && x.x > 0.0 && cloud_baked_flat_fetch(si, atmosphere_camera_hi + atmosphere_camera_lo + eye
				+ dir * x.x, cloud_view.params.w * x.x, true, f, t_col)) {
			color_t = vec4(f.xyz, 1.0 - f.w);
		}
		return;
	}
#endif
	vec2 top = cloud_sphere_crossing(ray, baked.sets[si].band.y);
	vec3 color = vec3(0.0);
	float transmittance = 1.0;
	float d0 = 0.0;
	float d1 = 0.0;
	if (top.x <= top.y && top.y > 0.0) {
		d0 = max(top.x, 0.0);
		d1 = top.y;
		vec2 bottom = cloud_sphere_crossing(ray, baked.sets[si].band.x);
		if (bottom.x <= bottom.y) {
			if (ray.h_c >= baked.sets[si].band.x) {
				if (bottom.x > 0.0) {
					d1 = min(d1, bottom.x);
				}
			} else {
				d0 = max(d0, bottom.y);
			}
		}
		d1 = min(d1, scene_m);
#ifdef CLOUD_BAKED_DEBUG
		{
			// debug: the grid map's round trip at the band segment's middle (m of error / 10 in red), the field there
			float tm = min(0.5 * (max(top.x, 0.0) + top.y), 20000.0);
			CloudBakedRay gd = cloud_baked_ray(si, eye, dir);
			float gx = gd.gx0 + tm * gd.gdx;
			float gy = gd.gy0 + tm * gd.gdy;
			float hm = cloud_height_along(ray, tm);
			float r = CLOUD_A_M + hm;
			vec3 q = gx * baked.sets[si].axis_x.xyz + gy * baked.sets[si].axis_y.xyz
					+ sqrt(max(r * r - gx * gx - gy * gy, 0.0)) * baked.sets[si].up0.xyz;
			vec3 o = q / ATM_STRETCH - (atmosphere_camera_hi + atmosphere_camera_lo);
			float err = length(o - (eye + dir * tm));
			color_t = vec4(err / 10.0, hm / 20000.0, 0.0, 0.0);
			int kd = cloud_baked_level(si, gx, gy);
			if (cloud_debug_height_m < 0.0 && kd < int(baked.sets[si].axis_y.w)) {
				// the baked field on the sphere of height -cloud_debug_height_m: ramp (red), scale (green), level (blue)
				vec2 hc = cloud_sphere_crossing(ray, -cloud_debug_height_m);
				float th = hc.x > 0.0 ? hc.x : hc.y;
				float gx2 = gd.gx0 + th * gd.gdx;
				float gy2 = gd.gy0 + th * gd.gdy;
				int k2 = max(cloud_baked_level(si, gx2, gy2), int(baked.sets[si].misc.z));
				vec2 a = texture(baked_a0, cloud_baked_uvw(si, min(k2, 5), gx2, gy2, -cloud_debug_height_m)).rg;
				color_t = vec4(cloud_baked_ramp(a.x), a.y / 100.0, float(k2) / 6.0, 0.0);
			}
			cloud_m = tm;
			return;
		}
#endif
	}
	bool volume = d1 > d0;
	float far_w;
	cloud_baked_set(0, ray, dir, eye, d0, d1, volume, scene_m, color, transmittance, cloud_m, far_w);
#ifdef CLOUD_BAKED_FADE
	{
		// A re-bake's swap: the set fading in too, the two renders cross-faded (premultiplied).
		float w = baked.sets[0].misc.y;
		vec2 top1 = cloud_sphere_crossing(ray, baked.sets[1].band.y);
		float e0 = 0.0;
		float e1 = 0.0;
		if (top1.x <= top1.y && top1.y > 0.0) {
			e0 = max(top1.x, 0.0);
			e1 = top1.y;
			vec2 bottom1 = cloud_sphere_crossing(ray, baked.sets[1].band.x);
			if (bottom1.x <= bottom1.y) {
				if (ray.h_c >= baked.sets[1].band.x) {
					if (bottom1.x > 0.0) {
						e1 = min(e1, bottom1.x);
					}
				} else {
					e0 = max(e0, bottom1.y);
				}
			}
			e1 = min(e1, scene_m);
		}
		vec3 color1;
		float transmittance1;
		float cloud_m1;
		float far_w1;
		cloud_baked_set(1, ray, dir, eye, e0, e1, e1 > e0, scene_m, color1, transmittance1, cloud_m1, far_w1);
		far_w = mix(far_w, far_w1, w);
		float a0 = 1.0 - transmittance;
		float a1 = 1.0 - transmittance1;
		float a = mix(a0, a1, w);
		cloud_m = a > 1.0e-6 ? mix(a0 * cloud_m, a1 * cloud_m1, w) / a : 0.0;
		color = mix(color, color1, w);
		transmittance = mix(transmittance, transmittance1, w);
	}
#endif
#ifdef CLOUD_BAKED_FAR_FALLBACK
	if (far_w > 0.0) {
		// beyond the flat geometry's cap (cloud_baked_flat_far_weight): the far field's share, behind the rest
		float dist_far;
		vec4 far = cloud_baked_flat_far_layer(ray, dir, eye, scene_m, dist_far);
		float a_far = transmittance * far_w * far.a;
		float a_rest = 1.0 - transmittance;
		cloud_m = a_rest + a_far > 1.0e-6 ? (a_rest * cloud_m + a_far * dist_far) / (a_rest + a_far) : 0.0;
		color += transmittance * far_w * far.rgb;
		transmittance -= a_far;
	}
#endif
	float alpha = 1.0 - transmittance;
#ifdef CLOUD_BAKED_DEBUG_ALPHA
	color_t = vec4(alpha, isnan(transmittance) ? 1.0 : 0.0, dot(color, vec3(1.0)) > 0.0 ? 0.5 : 0.0, 0.0);
	return;
#endif
	if (cloud_debug_output == 0) {  // (the surfaces' debug output 5 - cloud_check - shows no air loss over it)
		color -= cloud_air_shadow_loss(ray, dir, eye, scene_m, alpha > 1.0e-6 ? cloud_m : 1.0e30, transmittance);
	}
	color_t = cloud_stored_color(color, transmittance);
}
#endif
