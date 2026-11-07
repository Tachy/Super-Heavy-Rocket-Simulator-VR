# Eye adaptation, night vision and Moon light (M8)

History and the reasons behind superseded decisions: `docs/history/atmosphere-rendering.md`.

Plan `docs/superpowers/archive/plans/2026-10-24-m8-atmosphere.md`, spec
`docs/superpowers/archive/specs/2026-10-24-m8-atmosphere-design.md`.
Part of the atmosphere docs; index `docs/subsystems/atmosphere-rendering.md`. Tables and
shaders:
`atmosphere-scattering.md`.

## Model

- **Scene-referred adaptation** [Task 14 of the Moon surface plan, user 2026-11-05; `adaptation_meter.gd`, fidelity
  flag `Atmosphere.scene_referred_adaptation`, on in the game; the user argument `--analytic_adaptation` switches it
  off]: the eye adapts to the luminance of the field it sees, one rule for every scene. `AdaptationMeter`, a
  CompositorEffect (world.gd's Compositor only; the GPU check's has none), samples the HDR frame before the transparent
  pass (after the opaque pass, the sky and the cloud pass; MSAA resolved for it) on a sparse grid (~65 000 samples per
  view), composites the cloud pass's result over each sample as the transparent pass's composite quad does
  (`CloudEffect.frame_output_layer`: T x scene + colour), de-gains it to cd/m^2 (scene value / that frame's
  `scene_per_cd_m2`) and adds it, Gaussian-weighted toward the image centre (sigma 0.5 half diagonals
  [E: Moon & Spencer 1945, the surround's weight falls with eccentricity; call: from the image centre, no per-eye
  projection centre or solid-angle weight]), into two 128-bin log histograms (photopic,
  scotopic; 1e-6 .. 1e6 cd/m^2, bin 0 black). Both VR eyes go into one histogram (one adaptation, no eye difference).
  The directions beyond the frame are added before the statistics ("Field beyond the frame" below).
  The histograms come back asynchronously (`buffer_get_data_async`, every ~2-4 frames, no stall). The field luminance
  L_f (`field_log_average`): the weighted geometric mean, the brightest 2 % of the weight cut (Sun disc, glints [E]),
  bins more than 3.7 decades below the highest bin left excluded (no signal at that adaptation [S: Kunkel & Reinhard
  2010, simultaneous range ~3.7 log units]; a black sky beside lit ground does not pull it down; a field with nothing
  above black gives 0, the dark-adapted floor). The field is taken as its scene's mid grey [E: a convention with two
  readings - the photographers' 18 % grey card (a reflected-light meter calibrated to ISO 2720's K = 12.5 renders
  the metered luminance as a ~12-18 % grey, by maker) and Reinhard et al. 2002's display key 0.18 for the log
  average]: the adaptation
  luminance L_a = L_f / `FIELD_MID_GREY` (0.18), the scotopic one likewise, and everything downstream is the white-plate
  rule's (Ferwerda gains, mesopic k, CIECAM02 factors, star floor, pacing, clamp [1, 1e5], cap at L_ref). The white-plate
  rule assumed the field to be 18 % of the plate (a grey world); the field rule measures it: identical in a grey world,
  a darker field (regolith under a low Sun, the ocean from orbit) is seen brighter, a brighter one darker.
  A view jump (`snap_adaptation`) drops older measurements; the eye stays snapped
  to the new view's field while its terrain streams in (World sets `view_loading`) and at least 1 s
  (`FIELD_SNAP_MIN_S` [call]). Until the first measurement of a view, without a RenderingDevice, or 30 frames without
  one (`FIELD_STALE_FRAMES`), the white-plate rule below applies; the next measurement takes over paced (no step; only
  a new view snaps). The meter measures every 4th frame (`MEASURE_EVERY` [call]; only then it asks for the MSAA
  resolve). Display-referred emitters (vision.gdshaderinc `vision_display_referred`: plume, the stars' own
  brightness, orbit lines, labels, the VR panel) all draw in the transparent pass, after the meter: they are not
  counted. So are the tower lamps' lenses (`light_lens.gdshader`, blend_add, physical and scaled by the gain, also in the
  transparent pass: small, under the meter's 2 % cut; the lamp-lit surfaces enter through `lamp_adaptation_cd_m2`). Counted, their fixed display value D de-gains to D / gain and feeds the gain back on itself (log g_new =
  C + s log g, s their weight share; in a black field s = 1 and the gain runs to a clamp set by D);
  `adaptation_meter_test.gd` checks that every display-referred shader has a blend mode. Feedback slope measured with
  `render_check --gain_probe` (field at two held gains, s = -d ln L_f / d ln g): pad noon 0.0000, pad noon under a
  tau-15 overcast 0.0000 (field 2226 cd/m^2 both with the meter's own cloud composite and measured after the
  transparent pass), Apollo 11 0.033, lunar night 0.027 (both from the rocks drawn without the gain, open points);
  every start gain goes to the same target.
  Values (render_check, 2026-11-05, field L_f / white-plate L_a / gain; PNGs `screenshots/eye_adaptation/task14/`):
  Starbase viewpoint noon (Sun 48 deg, view east) 2475 / 37 624 / 3.14; dusk (Sun -3.8 deg) 3.99 / 43.8 (glow) / 1379;
  night (no Moon, lamps 2 km off) black / floor / 12 583; 400 km by day (view 20 deg down, limb haze) 2558 / 43 121 /
  3.03; IM-2 2 m (Sun 6.75 deg) 456-586 / 43 448 / 17.1; Apollo 11 down-Sun (Sun 14.5 deg) 661-712 / 41 064 / 11.2;
  lunar night under a nearly full Earth (0 N 45 E, 2026-11-07T14:00, view west) 0.033 / 5.71 (Earth in view) / 9334
  (rods 0.75). The Earth's day scenes measure ~6 % of the plate, not the grey world's 18 %. Pad noon in the camera mode
  (f/16 1/100 s ISO 100): ground (rows 0.80-0.98) 1874 cd/m^2, sky band (rows 0.05-0.30, ~110 deg from the Sun's
  azimuth) 3136 cd/m^2; a white horizontal plate there is ~29 000 cd/m^2 (from the 37 624 plate, Sun 48 deg, [E] ~15-20
  klux sky), so the ground draws as reflectance ~0.064 - the terrain shading is linear in the imagery's albedo and that
  imagery (NAIP z17 in the view sector, sRGB-decoded BT.709) is 0.051 mean / 0.048 median. The dark ground is the
  imagery's calibration, not the shading (open points). Apollo 11 regolith (AS11-40-5903, rectangle 0.65-0.95 x
  0.30-0.50): photo 147, render 144 (8-bit luma); left 0.02-0.30: photo 117, render 135 - a consistency check (photo
  and render are both field-metered, so they agree by construction), not a validation of the regolith's luminance.
  Cost (render_check --gpu_time 600, pad noon, RTX 4090): desktop 1920 x 1080 1.84 ms with the meter, 1.82-1.85 without;
  stereo mock (3534 x 2838 per eye) 4.23 ms with the meter (every 4th frame), 4.29 without; measuring every frame it
  was 4.54-4.55 (+0.25 ms, the extra MSAA resolve). Sky radiance cube with the view-following gain: a head nodding
  -30 .. +40 deg every 4 s redraws it 0.5 times per second (gain 2.97-3.30), and redrawing it every frame changes the
  stereo GPU time by < 0.02 ms.
- **Anchored rule (option; its anchor is half of the default rule D below)** [2026-11-07, `Atmosphere.field_rule`
  `FieldRule.ANCHOR`, user argument `--anchored_adaptation`]: the field rule's grey-world key over-brightens dark scenes and needs a hand key in the white cabin
  (cockpit_shots' 0.5 / 1.2), the white-plate rule is too dark outside and black in the cabin. Anchoring theory
  [S: Gilchrist et al. 1999, "An anchoring theory of lightness perception", Psychol. Rev. 106, 795: the highest luminance
  of a sufficiently large area is seen as white]: from the same histograms, after the same 2 % highlight cut and
  3.7-decade exclusion (`AdaptationMeter._field_bins`), the anchor L_anchor (`field_anchor`) is the weighted
  `ANCHOR_PERCENTILE` (0.95 [call]) percentile of the remaining weight, each bin's weight spread evenly in log across the
  bin (no step at a bin edge); L_a = L_anchor / `EyeAdaptation.ANCHOR_WHITE` (0.9 [E], the reflectance the anchor is seen
  as), the scotopic one likewise; `field_key` is not used. A black field gives 0 (the dark-adapted floor); everything
  downstream (Ferwerda gains and their low-light limits, pacing, mesopic k, CIECAM02 factors, clamps, cap at L_ref,
  fallbacks) is the field rule's. A bright spot under ~7 % of the weight (2 % cut + 5 % above the percentile) does not
  move the anchor, a larger bright area sets it (`adaptation_meter_test.gd`). Both statistics are measured under either
  rule (`last_field_cd_m2`, `last_anchor_cd_m2`; render_check and cockpit_shots log them). Comparison A field / B
  white plate / C anchored: `screenshots/eye_adaptation/anchored/` (values in the open points).
- **Highlight-protection rule D - the game's field rule** [user 2026-11-07, `FieldRule.PROTECT`, default;
  `--mean_grey_adaptation` / `--anchored_adaptation` select the others; `Atmosphere.field_adaptation_luminance`]: L_a = max(L_f / `field_key`, L_anchor /
  `ANCHOR_WHITE`) - the stricter (darker) of the mean-grey and the anchor exposures, the scotopic pair likewise; everything
  downstream unchanged. Intent: the anchored white where a bright area would clip, the mean grey elsewhere (dark scenes).
  Test: `adaptation_meter_test.gd`. Comparison A field / C anchored / D protected:
  `screenshots/eye_adaptation/protected/`: D equals the mean-grey rule in every outdoor scene (Moon day, lunar night,
  pad noon, dusk, 400 km, start view: the anchor / 0.9 is 2-4x the field, the field / 0.18 5.5x) and the anchored rule
  in the cockpit (gain 157 against 480 at key 0.18: anchor 242 cd/m^2 = 15x the field), so no hand key is needed there.
- **Field beyond the frame** [look fixes T4, 2026-11-07; `adaptation_surround.gd` `AdaptationSurround`, fidelity flag
  `AdaptationSurround.enabled`, user argument `--frame_only_adaptation` switches it off]. The meter sees only the frame:
  looking up into the noon sky the field was the sky alone and the gain doubled (pad noon el 5 / el 60: field 4642 /
  2354 cd/m^2, gain 1.67 / 3.30), the sky read pale. The eye's adaptation state integrates a wider surround (the
  periphery, eye movements). The field is now the frame plus the directions within `CAP_HALF_ANGLE_DEG` 70 deg of the
  view axis that the frame does not show [E: the monocular field reaches ~60 deg up, ~75 down, ~60 nasal, ~100 temporal,
  mean 74; 60 deg left the ground out of every view 60 deg up], weighted per solid angle by exp(-theta^2 / 2 sigma^2),
  sigma 30 deg [E: the continuation of the frame's own centre weighting per steradian for the 50 deg desktop frame:
  0.36 at its corner, 0.83 at its top edge against the Gaussian's 0.36 / 0.71; Moon & Spencer 1945's weight also falls
  with eccentricity]. In-frame weights stay as they are; the out-of-frame directions (rings of 10 deg, ~150 query
  directions, `in_view` against each rendered view's projection: in VR the union of both eyes) enter with the frame's
  total weight x their share of the angular weight, sum_out / sum_in. Their luminance: Godot's sky radiance cube is
  engine-internal (no script binding of `sky_get_radiance_texture_rd`), so the analytic model it is drawn from is
  evaluated on the CPU (`Atmosphere._surround_luminance`) into a store of 24 x 24 directions about the local vertical
  (7.5 deg bands, a band edge on the horizon; 8 cells per frame, a complete store swapped in every 0.8 s; a view jump
  builds a new one in frames of at most ~2 ms each, the frame alone meanwhile; while V = 0 the store ticks every 8th
  frame): the core's sky luminance
  along the ray (the Moon's penumbra segmented) or the model ground (Sun + sky, through the air) with the near
  imagery's albedo profile (GroundAlbedoNear, faded to the tables' ground albedo 1.5-6 km out, as the cube), x the
  cube's cloud factors (sky / ground half); near the Moon the regolith on the eye's ground sphere as the Moon-in-view
  term (both lights, grey), the sky black. Directions the model shows black add no weight (no signal). Enclosed views:
  the model knows no walls, so the out-of-frame part is weighted by a visibility V from the frame's field against the
  model's field over the in-frame directions - the same statistic (the model's in-frame rays as a histogram through
  `AdaptationMeter.field_log_average`: 2 % cut, 3.7-decade range): 1 within 0.3 decades, 0 beyond 0.6 (smoothstep)
  [call]. The disagreement in decades is low-passed with a 1 s time constant (`VISIBILITY_TIME_S`; a view jump
  restarts it), so panning across a structure or a hull does not fade the surround in and out; a black field counts as
  the histogram floor (1e-6 cd/m^2), so the measure is continuous (both black: 0 decades, open). The highlight cut, the 3.7-decade range, the anchor percentile and rule D run on the combined field
  unchanged; display-referred emitters stay out (the model has none). Test `game/tests/adaptation_surround_test.gd`
  (headless): the query set integrates the cap's weight (1 %), a uniform world keeps its field (1e-9), looking up the
  field rises and level it falls (they converge), an enclosed view and a black surround leave the histogram unchanged,
  two VR eyes show more of the cap than one, the store's cells round-trip, V changes by small steps over a sweep of
  the frame's field and a bright structure closes it over seconds, not at once, the model's in-frame field excludes a
  ground 4.5 decades down like the frame, the scotopic histogram gets the out-of-frame content, a rebuild with a dear
  luminance takes several frames. `game/tests/adaptation_surround_cost.gd` (headless, real atmosphere): the view jump's
  rebuild, worst frame / total: pad noon 2.0 / 3.7 ms (2 frames), Dallas totality 2024-04-08 (penumbra, 8 segments per
  ray, 56 us per cell) 2.4 / 32.0 ms (15 frames) - complete in one go the penumbra store would have cost 32 ms, three
  VR frames.
  Values (render_check / cockpit_shots, gain frame alone -> with the surround, field cd/m^2; sheets and logs
  `screenshots/look_fixes/t4/`, report `.superpowers/sdd/2026-11-07-look-fixes/t4-report.md`): pad noon view east el 5
  1.674 -> 1.839 (4642 -> 4226), el 60 3.301 -> 2.729 (2354 -> 2847), el 88 2.943 -> 2.800 (2640 -> 2775): looking up
  60 deg changes the gain x1.48 instead of x1.97; the model's field over the in-frame directions is within 1-13 % of
  the frame's there (2370 / 2640 / 4047 against 2354 / 2640 / 4642). Dusk (Sun -3.6 deg, view 250 deg el 5) 300 -> 415
  (the darker sky away from the glow). Apollo 11 lunar day down-Sun el 5 3.60 -> 2.98; el 60 12 650 (a black field)
  -> 2.98. Start view 1.401 unchanged (space around the Earth adds nothing; V 0.25 with the drawn clouds). Cockpit
  (`--field_key=0.18`): pilot_eye 157.39, hatch 58.43, GoPro 87.43, helmet 85.74 unchanged (V 0: frame 16-130 against
  the model's 1600-2400 cd/m^2). Stereo mock (both eyes, 3534 x 2838): el 5 1.689 -> 1.732, el 60 2.657 -> 2.581 (the
  headset's frame covers most of the cap: out-of-frame share 0.13 against the desktop's 0.46). Cost: no GPU work (pad
  noon median GPU time desktop 1.876-1.880 -> 1.878-1.894 ms, stereo mock 5.805-6.084 -> 5.782-5.815 ms, noise); main
  thread 0.09 ms per frame (store) + 0.12 ms per measurement (every 4th frame), a view jump's rebuild spread over frames
  (see the test above).
- **White-plate rule** (the fallback and the GPU check's / headless tests' rule; formerly the only one):
  [call 2026-10-25, `eye_adaptation.gd`, fidelity flag `Atmosphere.eye_adaptation_enabled`, on in
  the game, off in the GPU check's comparisons]: cone gain on every physical luminance (tables, Sun light, Moon, Sun
  disc, city lights; plumes and the stars' own brightness are display-referred). L_a = the largest of: down = a white
  plate at the eye under the transmitted Sun at normal incidence and the sky, (E_sun,n + E_sky) / pi x the visible
  fraction (+ the Moon's plate at night); under clouds (Task 9) (E_sun,n (T_s + mu0 T_sd) + E_sky T_d) / pi with the
  clouds' factors at the eye (`weather-clouds.md` "Light"; 1, 0, 1 with weather off); up = the Earth in view (`Atmosphere.adaptation_luminance`); the twilight
  glow. Cone gain = L_ref / (10^1.255 t_p(L_a)) in [1, 1e5], L_ref = the plate under the TOA Sun, t_p Ferwerda's cone
  t.v.i.: above ~80 cd/m^2 exactly the Weber gain L_ref / L_a, below it the cone threshold flattens (gain bounded at
  ~1.24e4 by the t.v.i. floor). Pacing [E]: light tau 0.5 s, dark 110 s (Pattanaik et al. 2000), exponential in log
  luminance; time step = max(frame time, simulated step) (paused: real time, time warp: a time-lapse, a clock jump or
  a view change: adapted at once); the photopic target is capped at L_ref before pacing. Values: pad noon gain 1.10,
  400 km by day 1.000 (GPU check: = L_ref / L_a within 1e-9), Sun -3 deg at the pad 557 (half the glow, 74.8 cd/m^2).
  - Up part: 13 directions below the eye in model space (nadir, a ring at 0.6 x the Earth disc's angular radius, a ring
    grazing the limb 10 km up), each the plate equivalent of what it shows, x the Sun's visible fraction there, times
    the view factor (R / r)^2; view-independent (VR-stable). While the Moon's penumbra touches the atmosphere the sky
    samples are segmented like the umbra fluence (8 segments; ~0.7 ms per evaluation, eclipse only). Outside an
    eclipse no shadow test runs (`penumbra_touches` first). Earth in view of a Dallas eye 2 m above the terrain:
    1.90 cd/m^2; in totality L_a is the glow. At 400 km on the 2024-04-08 umbra axis: down 0, up 2390 cd/m^2 (the
    same Earth uneclipsed: 20871), gain 17.7 (visible Earth in deep penumbra, visible fraction 0.03 / 0.10 / 0.33 /
    0.65 at 200 / 400 / 1000 / 2000 km from the umbra's centre; kept physical [call]).
  - Away from the Earth (eye above the top of the atmosphere only): down: transmittance 1 (or the grazing ray's), the
    visible fraction also takes the Earth's disc in front of the Sun (within the Sun's radius of the limb the penumbra
    is counted twice [call]); near the Moon the Sun's fraction at the eye is the lunar terrain's
    (`eye_terrain_sun_fraction`, `MoonSunHorizon`, see atmosphere-scattering.md) in place of the Moon's mean-sphere
    part. Up: the Earth in view x the part of its disc the Moon leaves. Third term, the Moon in
    view (`Atmosphere._moon_in_view_luminance`, needs `update(..., moon_gcrf_m)`): regolith luminance as the Moon
    shaders draw it - mean texture albedo 0.30 [D: `moon_color.png` area-weighted, sRGB-decoded, BT.709] x the regolith
    reflectance (R x LS x f(g) for the Sun, x the solar disc the Earth leaves at the ground point, and the same for the
    earthshine) - averaged cosine-weighted over the lunar cap (4 rings uniform in sin^2 x 8 azimuths) x (R / r)^2.
    The lunar ground sphere is the mean sphere, or 2 m below an eye standing below it. Values (measured with the regolith
    scale R 0.5126; since Task 11 fix C's albedo tie to the WAC w map R is 0.4408 at an unchanged mean albedo 0.30, so
    this analytic Moon term is 0.86 x these values; moon-surface.md, Hapke reflectance): lunar day 1.000 (Moon
    term 3264 cd/m^2, Sun plate 42 400); deep space halfway to the Moon 1.000; lunar night under a nearly full Earth
    (phase 10 deg): Earth in view 6.17 cd/m^2, Moon in view 0.20, earthshine 7.35 lux, cone gain 2845 (Weber 6872).
    At partial phases the 13-direction cap underestimates the Earth by up to ~2x (half Earth) [call: view-independent,
    no foveal term].
- **Earthshine.** `Atmosphere.earthshine_relative`: the Earth a Lambertian sphere of the Bond albedo 0.306 [S: NASA
  Earth Fact Sheet] (geometric albedo 2/3 A) at its phase angle a (Sun - Earth - Moon), illuminance on a plate at the
  Moon's centre facing the Earth / the TOA Sun at 1 AU = (2/3) A (R / d)^2 (sin a + (pi - a) cos a) / pi / d_AU^2.
  Full Earth at 384 400 km: 5.6e-5 = 7.46 lux (measured estimates ~7-15 lux). Clouds, sun glint, city lights not
  modelled. Once per frame in `Atmosphere.update`, global shader uniform `moon_earthshine` = DAY_EXPOSURE x that;
  `moon.gdshader` and `terrain_moon.gdshader` draw albedo x R x moon_earthshine x LS(mu_e, mu) x f(g_e); the reddened
  umbra glow is unchanged. The eye adaptation's Moon-in-view term and `last_earthshine_lux` use the same function.
  Display: the lunar night ground under a nearly full Earth (0 N 45 E, 2026-11-07T14:00, cone gain 3012) is dark grey;
  the Da Vinci glow of a 2-day crescent from Starbase at Sun -8 deg (2026-11-12T00:15, gain 5610-5670) lies 16 % above
  the twilight sky beside the Moon.
- **Night vision** (user decision 2026-10-25, "like the eye"; `eye_adaptation.gd`, `vision_effect.gd`,
  `vision.gdshaderinc`). Ferwerda, Pattanaik, Shirley & Greenberg 1996 (SIGGRAPH 96) [call: over Pattanaik 2000 /
  Kirk & O'Brien 2011]: Ward threshold matching with cone t.v.i. (Eq. 4: log t_p = -0.72 below log L_a -2.6, log L_a -
  1.255 above 1.9, (0.249 log L_a + 0.65)^2.7 - 0.72 between) and rod t.v.i. (Eq. 5: -2.86 below -3.94, log L_as -
  0.395 above -1.44, (0.405 log L_as + 1.6)^2.18 - 2.86 between), display = m_c L + k m_r L_s (Eq. 6). m_c = the cone
  gain, m_r = L_ref / (10^1.255 t_s(L_as)) [call]. A scene without light stays black however dark-adapted the eye.
  - k = 1 - s, s the rods' saturation of Jensen et al. 2000 ("Night rendering", UUCS-00-016, sec. 3.1): 0 below 0.01
    cd/m^2, 1 above 4, 3 x^2 - 2 x^3 with x = (log L + 2) / 2.6 between, on the photopic adaptation luminance [call].
  - Scotopic luminance L_s = w_s . rgb, w_s = (0, 1.4730, 0.9238) [D: `tools/validation/scotopic_rgb_weights.py`, CIE
    1951 V'(lambda), K'm 1700 lm/W, least squares in S/P over black bodies 2000-25 000 K, every weight >= 0; worst
    4.2 %, illuminant A +1.4 %, 6504 K -1.7 %; S/P of RGB white 2.40]. Non-negative because the post step is linear
    (no clamp). L_as is the chosen plate's colour through w_s.
  - Rods' image achromatic with a blue tint at chromaticity (0.25, 0.25) (Jensen et al. 2000; Thompson, Shirley &
    Ferwerda 2002, JGT 7(1)) [call].
  - Where: cone gain in the shaders as pre-exposure; rods by one CompositorEffect (`VisionEffect`, on the
    WorldEnvironment's Compositor in world.gd and the GPU check) after the transparent pass with the resolved MSAA
    colour, before glow and AgX: rgb' = C rgb + k g_r / g_c x (w_s . rgb) x tint, C = `EyeAdaptation.cone_map`: the
    white balance W (below), then Y + S (rgb - Y) (Y the BT.709 luminance); per pixel on every view layer; nothing
    dispatched while k g_r / g_c = 0 and C = I (the adaptation off).
  - Display-referred emitters write the inverse of that linear step (Sherman-Morrison): q = C^-1 c, then
    q - u (w_s . q), u = s C^-1 tint / (1 + s w_s . C^-1 tint) (`vision.gdshaderinc`, globals `vision_rod_weights`,
    `vision_inverse_tint`, `vision_cone_inverse`): plume, the stars' own brightness, the orbit ribbon, flight trail, the
    Moon's orbit line, map labels and the VR panel look as before. Holds under alpha and additive blending. GPU check:
    opaque `display_colour` quad at the moonless night (rod scale 138, S 0.52) = its day pixel within 0.5 % of its
    peak; half-transparent over a scene-referred background within 0.5 % (limit rgba16f; tolerance 2 % of the peak).
    Map labels (`MapLabel`) draw Label3D's black outline as eight black copies 2.5 font pixels out. The Sun disc
    clamps its scene value at 6e4 (the compute step also clamps and drops NaN).
  - No city lights (user 2026-10-29): the Earth's night side is the lighting alone (Moon, sky, twilight) on the
    day imagery; the Black Marble night texture and `earth_city_lights_cd_m2` are removed (former model:
    docs/history/atmosphere-rendering.md).
  - Glow (`Atmosphere._twilight_glow_luminance`, `_limb_glow_luminance`) [call: view-independent]: enters L_a at
    `GLOW_ADAPTATION_SHARE` 0.5 of the brightest sky. Twilight: Sun below about +5 deg (faded, see "Continuous through sunset"), brightest sky along the
    Sun's azimuth (elevations 1-40 deg, soft exclusion 8-14 deg from the Sun; azimuth-only sampling is an accepted limit).
    Solar eclipse at the eye (its own visible fraction < 1): the same over 8 azimuths x 1 / 3 / 6 / 12 deg of the
    eclipsed sky (`_eclipsed_in_scatter`). Orbit: above the atmosphere with the Sun behind the Earth and the eye not
    near the Moon (its disc smaller than the Earth's), the brightest limb toward the Sun (tangent heights 2-40 km).
  - Brightness and colourfulness (user decision 2026-10-25, `EyeAdaptation.brightness_factor` / `chroma_factor`):
    CIECAM02 luminance-level factor F_L [S: CIE 159:2004; Moroney et al. 2002, IS&T/SID CIC 10], F_L = 0.2 k^4 (5 L_A) +
    0.1 (1 - k^4)^2 (5 L_A)^(1/3), k = 1 / (5 L_A + 1); brightness Q ~ (A_w + 4) F_L^0.25, colourfulness M = C
    F_L^0.25. B = Q_w(L_a) / Q_w(L_day) applied as a luminance ratio, S = (F_L(L_a) / F_L(L_day))^0.25 on the cones'
    chroma at constant luminance; L_A = L_a x Y_b / 100, Y_b = 20; both 1 at and above L_day = 1e4 cd/m^2 [call]. B
    held below 79 cd/m^2 (Weber range), S below 4 cd/m^2 (mesopic limit). Values (L_a -> B, S): >= 1e4 -> 1, 1; 3264
    -> 0.79, 0.91; Sun -3 deg (90 cd/m^2) -> 0.378, 0.676; <= 79 -> B 0.368; <= 4 -> S 0.521. B multiplies the cone
    gain's pre-exposure (`luminance_scale`, `scene_per_cd_m2`, `atmosphere_exposure_gain`; the `exposure_gain` field
    stays the cone gain), so the star threshold (true luminance via `scene_per_cd_m2`) is unchanged; S acts in the
    post step (it runs every frame since the white balance, 958263d1).
  - Moon disc and regolith (`moon_regolith.gdshaderinc`, mirrored by `Atmosphere.regolith_reflectance`): one BRDF for
    every light on the regolith, albedo x R x E / pi x LS(mu0, mu) x f(g), LS = 2 mu0 / (mu0 + mu), f(g) = O(g)
    10^(-0.4 (0.026 g + 4e-9 g^4)) / Phi_LS(g), opposition surge O = 1.35 - 0.05 g below 7 deg [S: K&S 1991 as
    Schaefer's skycalc], Phi_LS(g) = 1 - sin(g/2) tan(g/2) ln cot(g/4) (`tools/validation/moon_disc_calibration.py`);
    g held at 150 deg beyond [call]. A normal facing away from the eye (mu < 1e-3) is taken onto the limb (its eye
    component raised to 1e-3) before mu0 [call, 2026-10-25]; GPU check (section 11, MSAA 2x): no pixel above 2 x the
    disc centre. R = 0.513 [D]: the full-phase disc of the texture's near side (projected-area mean albedo 0.2245; the
    sphere's mean 0.30) gives K&S's 0.313 lux (equivalent geometric albedo 0.115). Globals from `Atmosphere.update`:
    `moon_regolith_scale` (R), `moon_sun_scene` (DAY_EXPOSURE / d_AU^2), `moon_earthshine`.
  - Moonlight: the Moon's illuminance outside the atmosphere by Krisciunas & Schaefer 1991 (PASP 103, 1033: Eq. 9
    m = -12.73 + 0.026 |a| + 4e-9 a^4, Eq. 8 I = 10^(-0.4 (m + 16.57)) fc; full Moon 0.313 lux at 384 400 km), x the
    opposition surge, x the distance squared, x the Earth's shadow at the Moon (no red glow)
    (`Atmosphere.moonlight_relative`;
    relative to the TOA Sun at 1 AU: full Moon 2.4e-6, x 1.35 at a = 0). A second light of the solar spectrum through
    the same tables (`atm_moon_sky`, `atm_moon_at`, `atm_moon_sky_light`, `atm_moon_light`): moonlit sky, ground
    (terrain, Earth mesh, model ground) and sea (a second sea pass: glitter path, moonlit sky reflected), vehicles
    (world.gd `moon_light`, a second DirectionalLight without shadows, `Atmosphere.moon_light_at`), the adaptation.
    Active only while the Moon's plate is at least 0.1 % of the adaptation luminance. Left out [call]: the Moon's
    reddening (B-V 0.92), in-scattered moonlight between ground point and eye, the Moon's parallax (up to 1 deg).
  - Limits: one adaptation per view (no local / foveal adaptation: bright parts in a dim view still saturate - the
    full Earth from the lunar night, the full Moon's disc, city lights take the rods' tint); no loss of acuity, no
    glare; rods' slower dark adaptation (~30 min) paced like the cones' (110 s); no airglow, zodiacal light or
    integrated starlight (a moonless sky is black); Jensen's k on the adaptation luminance, not per pixel; B and S one
    value per view, no Bezold-Bruecke hue shift (lunar day keeps B = S = 1).

- **White balance: chromatic adaptation to the Sun's white** [look fixes T2, 2026-11-07; `EyeAdaptation.white_balance`,
  `cone_map`, `Atmosphere.white_balance`; fidelity flag `EyeAdaptation.white_balance_enabled`, render_check
  `--white_balance=off`]. The renderer's colours are colourimetric linear sRGB (white D65), but the Sun above the air is
  warmer: the core's TOA spectrum in linear sRGB is (1.123, 1, 0.961), B/R 0.856. Without adaptation a sunlit white cloud
  seen from space was drawn salmon (start view: cloud radiance B/R 0.79-0.84, physical - the TOA Sun x a cloud top's
  near-flat reflectance under ~0.2 Rayleigh depth; displayed B/R 0.95-0.96 against EPIC's neutral (238, 243, 241)). The eye
  adapts to the light it is in: a von Kries transform in CAT02's cone space [S: CIE 159:2004; Moroney et al. 2002] maps
  the adopted white to the display's grey of the same luminance, W = M_rgb^-1 M_CAT02^-1 diag(grey / white) M_CAT02
  M_rgb (diagonal-dominant: R x0.970, B x1.056, the green row 1.020). Adopted white [call]: the Sun's own colour above the
  atmosphere (from the tables' `solar_illuminance_rgb` x `sun_radiance_to_luminance`), the same at every place and
  hour - the daylight white: a sunlit white surface in space is white, light reddened by the air (low Sun, terminator,
  sunset) keeps its colour (no adaptation to a sunset's own light), the ground's daylight (Sun + sky, CCT ~5500-6500 K)
  is within ~5 % of it. Full adaptation (CIECAM02's D = 1, reached above ~100 cd/m^2; at night the rods dominate). With
  the eye adaptation off (fixed exposure, `--camera_exposure`) the image stays colourimetric. Cost: the post step now
  runs every frame (it was skipped by day): pad view 1600 x 900 not measurable (median 1.89 vs 1.89 ms), stereo mock
  3534 x 2838 per eye +0.11 / +0.19 ms (5.70 / 5.80 vs 5.59 / 5.61 ms, two rounds). Test `game/tests/white_balance_test.gd` (headless): the D65 grey kept
  (identity), the TOA Sun B/R 0.856 in 0.80-0.90 and drawn grey at its luminance (1e-6), the Sun through Rayleigh air
  mass 4 at 700 hPa still warm (B/R 0.485 -> 0.564 < 0.75), post step o display-referred inverse = identity (1e-6).
  A/B numbers: `.superpowers/sdd/2026-11-07-look-fixes/t1-t2-report.md`.

- **Lamps in view** [call 2026-10-31, `Atmosphere.lamp_adaptation_cd_m2`]: one more candidate of L_a's maximum (white, and its scotopic
  equivalent), set by `World` from `TowerView.lamp_adaptation_cd_m2`: 3 % of the lit pad's luminance (Pad B deck 16.8 cd/m^2 at night) x the
  camera's reach (300 m full, 0 at 1.5 km) x the view axis (25 deg full, 0 at 60 deg). 0 by day and at sites without lamps. Details
  `launch-structures.md` "Lights".

- **Ambient and the gain.** The sky's radiance cubemap holds scene values, so it is redrawn at a 2 % change of
  `luminance_scale`; during a fast light adaptation (tau 0.5 s) the ambient lags the gain by the 8 to 16 frames of the redraw
  (no scaling instead: `atmosphere-scattering.md`, "Why the gain needs a redraw").
  Pad by day: gain 1.1, shade side of a white hull `73 89 105` beside a lit side of 132-221. Dark adapted, gain 12 602,
  moonless pad `0 0 0`: no ambient is invented.

- **Continuous through sunset** (user approved 2026-10-31). The twilight glow has no on / off gate and no hard cutoff:
  each sample's luminance is weighted by a smoothstep of its angle from the Sun (0 up to `TWILIGHT_GLOW_MIN_SUN_DEG` 8
  deg, 1 from `TWILIGHT_GLOW_FULL_SUN_DEG` 14 deg [E]) before the maximum, and the whole term fades with the Sun's
  elevation (1 up to +3 deg, 0 from +8 deg, `TWILIGHT_GLOW_FADE_START_DEG` / `_END_DEG` [E]; the eye's own eclipse
  keeps it at any elevation). The fade is needed because by day the aureole itself is brighter than the plate (38 000
  cd/m^2 at Sun 15 deg); from +3 deg on the plate is the larger term, so no day value changes. Rule test
  `game/tests/sunset_adaptation_test.gd` (headless): Sun +8 .. -18 deg in 0.1 deg steps at the pad and 20 km up; the
  largest step of ln target / gain per step 0.16 / 0.19 (limit 0.35, natural fall of the darkest twilight) and the
  largest second difference of ln glow 0.07 / 0.13 (limit 0.15, the floor's kink included; before: 0.9 / 1.4 and 13 / 12).
  Pad values (target cd/m^2, gain): Sun +1 deg 804 (53), 0 deg 410 (103), -2 deg 126 (337), -3 deg 75 (557), -6 deg 5.7
  (2950), -12 deg 0.04 (11 200). `sky_radiance_check --timelapse` counts no gain step and leaves no frame out. Limits:
  the glow now exceeds the plate from about Sun +2 deg (the sky near the horizon, half of it, against a reddened
  Sun's plate); the aureole inside 8-14 deg is under-weighted by the fade (a smoothstep, not a glare model).

## Standing decisions

- Eye model: Ferwerda 1996 cone/rod t.v.i. with Jensen 2000 rod saturation, CIECAM02 brightness / chroma terms,
  Schaefer 1990 star threshold (user decision 2026-10-25, "like the eye").
- Field statistic: highlight protection D, the darker of the mean-grey and the anchor exposures (user 2026-11-07,
  after comparing mean grey, white plate, anchored; the mean grey alone washed out the cabin, the white plate was too
  dark outside and black in the cabin, the anchor alone washed out sky / limb scenes). Next step offered: local
  (retinal) adaptation for small bright objects (the Earth from the lunar night still clips).
- Scene-referred adaptation to the measured field (user 2026-11-05, Task 14): one rule for every scene, one value for
  both VR eyes (formerly view-independent; the white-plate rule is the fallback). Earth-in-view stays physical (no
  display-side highlight treatment). Display-referred emitters never set the adaptation (the meter runs before the
  transparent pass; Task 14 fix round 1, review I-1).
- The field is wider than the frame (look fixes plan, user 2026-11-07 "Alles wie vorgeschlagen umsetzen", proposal F4):
  70 deg about the view axis from the analytic surround model, gated by its agreement with the frame (enclosed views
  keep the frame alone). Rule D, the highlight cut and the anchor percentile unchanged.
- City lights scene-referred with scale [E] (user ruling 2026-10-25).
- White balance to the TOA Sun's white, fixed (not to the plate's current light: a sunset stays red) (look fixes plan,
  user 2026-11-07 "Alles wie vorgeschlagen umsetzen", proposal F7).

## Open points and known limits

- Earth in view for a Moon eye at partial phases underestimates up to ~2x; the eclipse-time Earth renders several
  times its day brightness (AgX rolls it toward white) - kept physical.
- Earth in view and the earthshine ignore the clouds the Earth is drawn with (Task 10: the scaled-space Earth shows
  them): the eye adapts to the cloud-free model ground (the Sun's plate dominates wherever the eye is sunlit; on the
  lunar night side the cloudy Earth is brighter than adapted to); the earthshine's Bond albedo 0.306 holds the clouds
  on average. Follow-up: the 13 directions through the cloud column.
- Eclipsed-Moon corona not modelled; no per-pixel / foveal adaptation.
- Field beyond the frame (T4): the surround model knows no structures, vehicles, terrain relief, drawn cloud shapes
  (only the cube's uniform cloud factors) or the Earth's disc in the lunar sky; an open view next to a tower or a hull
  counts open sky / ground there. The model's regolith is ~2x the rendered one (Apollo 11 view: model 4375 against frame
  2169 cd/m^2 over the same directions, the open point of the Moon shading below; still inside the visibility's x2), so
  on the Moon the surround brightens the field. A lit surround of less than the 2 % highlight share is cut whole (lunar
  day 60 deg up: 2.4 % of the weight is regolith, just above the cut; a narrower lit rim would leave the black field).
  The store's state (Sun, cloud factors, eye, local vertical) is frozen at the start of each rebuild, so the swapped-in
  store is up to 0.8 s old (a view jump's: a few frames, a penumbra's ~0.2 s): fine on the ground, in an ascent or an
  orbit (altitude and Sun angle changing fast) the surround lags by that much; not refreshed on large changes.
  The visibility's thresholds (x2 / x4) are a call, not a measurement; the desktop runs at a fixed head, the "saccade"
  part of the eye's integration is the cap's Gaussian, not a temporal memory.
- Scene-referred rule (Task 14): `moon_rock.gdshader` (unshaded) does not multiply by `atmosphere_exposure_gain`, so
  drawn rocks stay at gain 1 while the regolith follows the gain (dark rocks on the lunar day at gain 11-17, black at
  night); the rocks also bias the field measurement low (IM-2: 586 -> 456 cd/m^2). Fix: `ALBEDO *= atmosphere_exposure_gain`
  as terrain_moon does (the file was in another task's hands, not changed here).
- No per-pixel / foveal term in the field rule: a small bright object (a gibbous Earth of ~2 deg in a lunar-night
  view, a distant plume) stays inside the 2 % highlight cut, so a gaze at it does not adapt to it.
- The VR panel was not in the stereo mock's lunar-night view (`--keep_vr_panel`), and the night plume seen from
  500 m (fov 25 deg) stays inside the 2 % highlight cut; the run-away the meter now avoids was therefore not
  reproduced in a render, only excluded by construction (display-referred shaders in the transparent pass, test).
- Anchored rule alone (2026-11-07, render_check / cockpit_shots, `--hide_ui`, PNGs and sheets
  `screenshots/eye_adaptation/anchored/`; field L_f / anchor L_anchor cd/m^2, gain A field / B white plate / C anchored):
  Apollo 11 day 1345 / 2710, 5.80 / 1.00 / 14.4 (ground sRGB luma mean 156 / 71 / 190); lunar night 0 N 45 E 0.045 /
  0.063, 8844 / 3012 / 10 843; Starbase noon 4526 / 8013, 1.72 / 1.15 / 4.85; dusk (Sun -3.6 deg) 19.6 / 86.0, 397 / 778
  / 452; 400 km day 2638 / 5649, 2.95 / 1.00 / 6.88; cockpit pilot_eye (A at the game key 0.18, not the shot's 0.5)
  15.9 / 242, 480 / 1.98 / 157. The anchor is only 2-4.4x the field in these centre-weighted fields, so C is brighter
  than A outdoors everywhere; at noon and from orbit the anchor is the sky / the hazy limb (a self-luminous area seen
  as a white surface), which washes the ground out. Open: percentile, ANCHOR_WHITE, a surface-only anchor.
- The rendered Apollo 11 regolith (490 cd/m^2 in the foreground probe, field 660-710) is 3-6x below the core's Hapke
  I/F x E_sun,n / pi at that geometry (I/F 0.08-0.10 at 25-30 deg down-Sun -> 3400-4300 cd/m^2 with the highland w
  0.429); the photo's f/11 1/250 ISO 160 exposure (mid grey 2363 cd/m^2) also points to ~2000+ cd/m^2. The field rule
  hides this (it normalises the field), the camera mode does not: an open point of the Moon shading, not of the
  adaptation.
