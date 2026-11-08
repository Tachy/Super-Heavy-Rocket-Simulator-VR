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
  L_f (`field_log_average`): the weighted geometric mean, the brightest 2 % of the lit weight (bins above black) cut (Sun disc, glints [E]; of the lit weight, not of the whole field, user 2026-11-07),
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
  total weight x their share of the angular weight, sum_out / W_in. W_in, the frame's angular weight, is integrated
  over the frame itself (`AdaptationSurround.frame_weight`: each view's frustum on 24 x 24 NDC cells, solid angle x the
  Gaussian, the union of the eyes; T5b fix round 1, review I4), and the out-of-frame query weights are scaled to the
  rest of the cap's integral - before, W_in came from the query directions inside the frame, so a frame narrower than
  their 10 deg spacing (fov 2: the full Moon) had W_in 0 and the surround dropped out ("model in frame -1"). A frame
  with fewer than 3 query directions inside (`MIN_QUERIES_IN_FRAME`, under ~15 deg) cannot be compared with the model
  (its field may be the Moon's disc, which the model does not draw): it keeps the visibility judged last, a fresh view
  starts open [call]; the model's in-frame field is then the store at the view axis (diagnostic). Their luminance: Godot's sky radiance cube is
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
  luminance takes several frames, the frame weight of the 50 deg frame within 1 % of a fine integral and of a 2 deg
  frame its solid angle, a 2 deg frame of a bright disc keeps the surround (share > 0.99, V open, its field the
  surround's within 2 %). `game/tests/adaptation_surround_cost.gd` (headless, real atmosphere): the view jump's
  rebuild, worst frame / total: pad noon 2.0 / 3.7 ms (2 frames), Dallas totality 2024-04-08 (penumbra, 8 segments per
  ray, 56 us per cell) 2.4 / 32.0 ms (15 frames) - complete in one go the penumbra store would have cost 32 ms, three
  VR frames.
  Values (render_check / cockpit_shots, gain frame alone -> with the surround, field cd/m^2; sheets and logs
  `screenshots/look_fixes/t4/`, report `.superpowers/sdd/2026-11-07-look-fixes/t4-report.md`): pad noon view east el 5
  1.674 -> 1.839 (4642 -> 4226), el 60 3.301 -> 2.729 (2354 -> 2847), el 88 2.943 -> 2.800 (2640 -> 2775): looking up
  60 deg changes the gain x1.48 instead of x1.97; the model's field over the in-frame directions is within 1-13 % of
  the frame's there (2370 / 2640 / 4047 against 2354 / 2640 / 4642). Dusk (Sun -3.6 deg, view 250 deg el 5) 300 -> 415
  (the darker sky away from the glow). Apollo 11 lunar day down-Sun el 5 3.60 -> 2.98; el 60 12 650 (a black field)
  -> 2.98, since the lit-weight cut (2026-11-07) 1.93 (the Earth's disc near the view centre enters the field). Start
  view 1.401 unchanged by the surround (space around the Earth adds nothing; V 0.25 with the drawn clouds), since the
  lit-weight cut 1.269: the cut is 2 % of the ~30 % lit weight instead of the whole frame's, so more of the bright cloud
  stays in the field (renders `screenshots/look_fixes/t5b/`). Cockpit
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
    white balance W (below), then Y + min(S, 1) (rgb - Y) (Y the BT.709 luminance); per pixel on every view layer;
    nothing dispatched while k g_r / g_c = 0 and C = I (the adaptation off). Then the display tone step (below).
  - Display-referred emitters write the inverse of that linear step (Sherman-Morrison): q = C^-1 c, then
    q - u (w_s . q), u = s C^-1 tint / (1 + s w_s . C^-1 tint) (`vision.gdshaderinc`, globals `vision_rod_weights`,
    `vision_inverse_tint`, `vision_cone_inverse`): plume, the stars' own brightness, the orbit ribbon, flight trail, the
    Moon's orbit line, map labels and the VR panel look as before. Holds under alpha and additive blending. Before it
    the emitter takes the display tone step's inverse (`vision_tone_inverse`, `vision_hunt_inverse`; globals
    `vision_tone`, `vision_tone_white`, `vision_hunt`, neutral while the step is off): AgX's colour of c scaled to the
    scene luminance the step's curve shows at its luminance, then the OkLab chroma / S - shown as AgX alone showed it,
    within ~1 % for an emitter alone on its pixel (CPU within 0.7 % per channel, GPU patches within 2e-3; the Hunt
    inverse keeps the OkLab lightness while the curve reads the luminance), approximate where
    it blends with the scene (the step is not linear). GPU check:
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
  - Brightness and colourfulness (user decision 2026-10-25, `EyeAdaptation.brightness_factor` / `chroma_factor`; S's
    reference look fixes T5, 2026-11-07): CIECAM02 luminance-level factor F_L [S: CIE 159:2004; Moroney et al. 2002,
    IS&T/SID CIC 10], F_L = 0.2 k^4 (5 L_A) + 0.1 (1 - k^4)^2 (5 L_A)^(1/3), k = 1 / (5 L_A + 1); brightness Q ~ (A_w +
    4) F_L^0.25, colourfulness M = C F_L^0.25; L_A = L_a x Y_b / 100, Y_b = 20. B = Q_w(L_a) / Q_w(L_day) applied as a
    luminance ratio, 1 at and above L_day = 1e4 cd/m^2 [call: the display cannot be brighter than its white], held
    below 79 cd/m^2 (Weber range). S = (F_L(L_a) / F_L(L_display))^0.25: the colourfulness the adapted eye sees matched
    on a display whose viewer is adapted to its white L_display = 100 cd/m^2 (`DISPLAY_WHITE_CD_M2` [S: ITU-R BT.2035
    reference display white; sRGB's reference 80 cd/m^2]; the same for the HMD [call]) - CIECAM02's corresponding
    colours; held below 4 cd/m^2 (mesopic limit), at most `CHROMA_FACTOR_MAX` 1.7 [call: guard; L_a is capped at the
    reference plate, S <= 1.66]. Values (L_a -> B, S): 3.8e4 (pad noon plate) -> 1, 1.64; 1e4 -> 1, 1.47; 3264 -> 0.79,
    1.34; 181 -> 0.44, 1.05; 100 -> -, 1; Sun -3 deg (90) -> 0.378, 0.99; <= 79 -> B 0.368; <= 4 -> S 0.77. B
    multiplies the cone gain's pre-exposure (`luminance_scale`, `scene_per_cd_m2`, `atmosphere_exposure_gain`; the
    `exposure_gain` field stays the cone gain), so the star threshold (true luminance via `scene_per_cd_m2`) is
    unchanged. S acts on the cones' image in two places: its part <= 1 linearly in the post step (Y + S (rgb - Y),
    `cone_map`; the display-referred inverse exact), its part > 1 in the display tone step (OkLab chroma x S at
    constant lightness and hue, `display_hunt`, below) - a linear boost about Y turns saturated blues 4-7 deg in OkLab
    hue. Fidelity flag `EyeAdaptation.hunt_display_reference` (render_check / cockpit_shots `--hunt=day`: the former
    reference L_day, S <= 1; docs/history/atmosphere-rendering.md). S is CIECAM02's colourfulness match to first order
    (C's t^0.9 and the cone compression ignored: OkLab chroma x S ~ CIECAM02 C x S^0.9). [call] S's reference is the
    display white 100 cd/m^2 also at dusk / night, where B dims the display (B 0.37-0.44): the display viewer's own
    adapting white would then be ~B x 100 cd/m^2 and S ~7 % higher (dusk ~1.00 -> ~1.07); kept at 100 pending the
    user's verdict on the dusk / night S (T5 report, concern 3).
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
    Active only while the Moon's plate is at least 0.1 % of the adaptation luminance. Its colour: the regolith's on the
    Sun's (`moon_light_colour`, global `atmosphere_moon_colour`, luminance kept; T5b fix round 1). Left out [call]:
    in-scattered moonlight between ground point and eye, the Moon's parallax (up to 1 deg).
  - Limits: one adaptation per view (no local / foveal adaptation: bright parts in a dim view still saturate - the
    full Earth from the lunar night, the full Moon's disc, city lights take the rods' tint); no loss of acuity; glare
    only of the Sun (the Moon's and the Earth's disability glare not modelled); rods' slower dark adaptation (~30 min) paced like the cones' (110 s); no airglow, zodiacal light or
    integrated starlight (a moonless sky is black); Jensen's k on the adaptation luminance, not per pixel; B and S one
    value per view, no Bezold-Bruecke hue shift (lunar day keeps B = 1, S 1.66).

- **White balance: the adopted white follows the scene** [look fixes T2, 2026-11-07; T5b fix round 1, user 2026-11-08;
  `EyeAdaptation.white_balance`, `degree_of_adaptation`, `advance_white`, `cone_map`; `Atmosphere.field_illuminant`,
  `white_balance`; fidelity flags `EyeAdaptation.white_balance_enabled` (render_check `--white_balance=off`) and
  `EyeAdaptation.adopted_white_follows_scene` (`--white_balance=toa`, render_check and cockpit_shots: the former fixed
  white of the Sun above the air at D = 1)].
  - Field illuminant (`Atmosphere.field_illuminant`, linear Rec.709 at luminance 1): the light reaching the scene,
    estimated physically from the analytic terms, not a grey world of the frame - of the plate at the eye (inside the
    air the HORIZONTAL plate, the global illuminance E_sun,n mu0 (T_s + T_sd) + E_sky T_d through air and clouds, at
    night + the Moon's horizontal plate and the moonlit sky; above the air the Sun at normal incidence; + the tower
    lamps' lit surfaces in their lumen-weighted CCT colour, `TowerLights.deck_light_colour`)
    and, above the air, the Earth's side (near the Moon the earthshine in the Sun's colour, as the shaders draw it;
    elsewhere the Earth in view), the brighter. So: the Sun above the air in space and on the Moon, Sun + sky on the
    ground, moonlight at night, the twilight sky at dusk. An enclosure the eye is in (`interior_light_colour`; World:
    the cabin of a `CockpitView` that `contains` the camera, its lamps' `LAMP_COLOUR`, D65) takes over as the surround's
    visibility V closes: V x outside + (1 - V) x interior. A plate below `FIELD_ILLUMINANT_MIN_CD_M2` 1e-3 cd/m^2 [call]
    (a moonless night) has nothing to adapt to: the white is held.
    Why horizontal (fix round 2, review I1): CIECAM02's adopted white is the illuminant of the scene; the scene is
    mostly horizontal (ground, deck, sea) and daylight is measured as global horizontal light [S: CIE 015:2018 daylight;
    Judd et al. 1964; Hernandez-Andres et al. 2001, JOSA A 18, 1325]. The luminance plate (Sun at normal incidence +
    horizontal sky, below) is unchanged; as a colour it weighted the Sun 1 / mu0 too much (1.35 at Sun 48 deg). Units
    checked (headless print at the pad): `sky_lux` and E_sun are both lux (sky 15 834 lx, Sun 74 646 lx horizontal /
    100 446 lx normal at Sun 48 deg; sky 17.5 % of the horizontal), so the sky had its weight; the white was warm
    because the direct beam is (AOD 0.1 at 550 nm, air mass 1.35: T 0.84 / 0.74 / 0.60, B/R 0.612, 4940 K = D50).
    Illuminant B/R and CCT [linear Rec.709, McCamy]: pad Sun 90 deg 0.784 / 5620 K, 60 deg 0.772 / 5570 K, 48 deg
    0.758 / 5510 K (was 0.724 / 5370 K), 30 deg 0.718, 15 deg 0.654, 5 deg 0.753 (sky-dominated; was 0.181), 0 deg
    1.52, dusk -5.5 deg 2.93 (unchanged, sky only); 400 km and the Moon 0.856 / 5920 K (the TOA Sun, unchanged).
    Reference: D55 = B/R 0.745, D60 0.874, D65 1.0 in linear Rec.709; measured clear-sky global daylight at high Sun
    ~5500-6500 K - the model sits at the band's warm edge [call: three-wavelength spectrum, AOD 0.1]. Known limit: an
    eye high in the air with the Sun below its local horizontal but still visible (the horizon dip) takes the sky's
    colour only (the scene below it is in twilight).
  - Degree of adaptation [S: CIE 159:2004; Moroney et al. 2002]: D = F (1 - (1 / 3.6) exp((-L_A - 42) / 92)), L_A = the
    paced L_a x Y_b / 100 (Y_b 20), F = `SURROUND_F` 1.0: the scene observer's surround is the scene itself, CIECAM02's
    "average" (S_R >= 0.2; dim 0.9 and dark 0.8 are films / slides in a dim / dark room) [S: CIE 159:2004 Table 1;
    Fairchild 2013, Color Appearance Models 3rd ed., 16.4]; the display observer fully adapted to the display white
    [call]. D 0.824 at a moonlit night, 0.90 in the lamp-lit cabin (L_a ~270), 1.000 by day and in space.
  - Transform: von Kries in CAT02's cone space with CIE 159's partial form, gains D grey / white + 1 - D (grey: the
    display's grey at the illuminant's luminance), W = M^-1 diag(gains) M - D = 0 the identity (the display white
    D65), D = 1 the illuminant drawn grey.
  - Pacing [S: Fairchild & Reniff 1995, JOSA A 12, 824: ~90 % after ~60 s; Rinner & Gegenfurtner 2000]:
    `CHROMATIC_TIME_CONSTANT_S` 26 s (one exponential in the log gains, 90 % at 60 s), snapped with the luminance's
    snap (a view or clock jump). The display-referred emitters keep the exact inverse (`vision_cone_inverse` from the
    same W).
  - Moonlight's colour (`Atmosphere.moon_light_colour`, global `atmosphere_moon_colour`): the moonlit sky, ground, sea
    and vehicles take the regolith's mean colour on the Sun's light (`moon_grey_tint`, luminance kept: K&S's moonlight
    stays the photometric value) - the full Moon's B-V 0.92 against the Sun's 0.65, before left out [call].
  Numbers, A (HEAD 3dd8340a) / B (this round), display sRGB of named regions, `screenshots/look_fixes/t5b_fix1/`
  (numbers.txt, sheet_*_ab.png): see the T5b report's "Fix round 1" (`.superpowers/sdd/2026-11-07-look-fixes/t5b-report.md`).
  Test `white_balance_test.gd` (below).
  Why a white balance at all (T2): the renderer's colours are colourimetric linear sRGB (white D65), but the Sun above
  the air is warmer: the core's TOA spectrum in linear sRGB is (1.123, 1, 0.961), B/R 0.856. Without adaptation a sunlit
  white cloud seen from space was drawn salmon (start view: cloud radiance B/R 0.79-0.84, physical; displayed B/R
  0.95-0.96 against EPIC's neutral). With the eye adaptation off (fixed exposure, `--camera_exposure`) the image stays
  colourimetric. Cost: the post step runs every frame: pad view 1600 x 900 not measurable, stereo mock 3534 x 2838 per
  eye +0.11 / +0.19 ms (T2). Test `game/tests/white_balance_test.gd` (headless): the D65 grey kept (identity), the TOA
  Sun B/R 0.856 drawn grey at its luminance (1e-6), the Sun through Rayleigh air mass 4 at 700 hPa still warm at D = 1
  toward the TOA Sun (B/R 0.485 -> 0.564), post step o display-referred inverse = identity (1e-6, also at D 0.82 toward
  the moonlight); D = 0.8240 at L_A 0, 0.9022 at a 270 cd/m^2 white, 1 by day, increasing; D 0 the identity, gains
  linear in D; the full Moon's light (Sun x regolith colour x air at el 83) adapted with the night's D: B/R 0.874 (pale
  cream, band 0.80-0.92; D 1: 1.000; the former TOA white 0.505; colourimetric 0.436); the moonlight colour keeps the
  Sun's luminance (1e-6); a D65 cabin lamp neutral (1e-6); pacing one time constant e^-1 (1e-9), 90 % in 60 s (0.0995),
  a snap jumps; the field illuminant at the pad at noon (Sun 48 deg) = the horizontal Sun + sky (1e-6) with its CCT in
  5200-6500 K (5510 K; [S] Hernandez-Andres et al. 2001 clear-sky global daylight, lower edge [call] -300 K for the
  model's spectrum). T2's A/B numbers: `.superpowers/sdd/2026-11-07-look-fixes/t1-t2-report.md`.

- **Sun glare: the disc masked, its disability glare a veil** [look fixes T5b fix round 1, user 2026-11-08;
  `AdaptationMeter` (shader mask, `sun_mask_parameters`), `Atmosphere.glare_veil`; fidelity flags
  `Atmosphere.glare_enabled` / `AdaptationMeter.sun_mask_enabled`, render_check `--glare=off`]. Before, the Sun's disc
  entered the histograms and only the 2 % highlight cut removed it: whether a disc alone in a black field was cut (the
  eye dark-adapted next to the Sun, stars beside it) or set the field depended on its pixel share (fov, resolution,
  desktop vs headset), and a Sun just outside the frame did not count at all.
  - Mask: the meter's compute skips every sample whose view ray lies within `SUN_MASK_RADIUS_FACTOR` 1.2 [call: limb
    and the sparse grid] x the Sun's apparent radius of its direction (World `_adaptation_view`: "sun", "sun_mask_cos";
    per rendered view the ray (x a + b, y c + d, -1) from that eye's projection and the Sun in that eye's axes: both VR
    eyes). Two counters after the histograms: the masked weight and its part at least `SUN_SEEN_SHARE` 0.05 of the
    disc's own (de-gained) luminance (`sun_seen_cd_m2`: the direct beam's normal illuminance / the disc's solid angle,
    or the shader's 6e4 clamp): the disc seen, not hidden behind something.
  - Veil [S: Stiles 1929, Holladay 1926; CIE 146:2002, the Stiles-Holladay disability glare formula]: L_veil = 10 E_gl
    / theta^2 (cd/m^2, E_gl lux at the eye in the plane normal to the line of sight, theta deg from the view axis; the
    form is stated for 1-30 deg, [call] extended: theta held at the Sun's radius 0.267 deg, beyond 30 deg it falls with
    E_gl = E_n cos theta to 0 at 90 deg - nothing from behind). E_n = `last_sun_normal_lux`, the direct Sun's normal
    illuminance at the eye (TOA / d^2 x the transmittance x the clouds' direct beam x the visible fraction). Whether it
    reaches the eye: with the Sun's direction in a rendered view the meter's seen share (seen / (mask x 1/1.44 x
    `SUN_SEEN_FULL_SHARE` 0.5 [call: the sparse grid puts ~4-6 samples into the mask, measured 0.48 for an open Sun]),
    at most 1); outside the frame the surround's V (open 1; an enclosure, whose walls the model does not know, 0).
    Scotopic veil: the direct Sun's colour through the scotopic weights. The veil lies over the whole retinal image, so
    the field's statistics take it as L_f + L_veil and L_anchor + L_veil (a black field becomes the veil; added per
    sample instead, a weak veil brought the black sky back into the mean and lowered L_a: 400 km, Sun 70 deg off axis,
    2.0e4 -> 6.5e3 in a trial render). theta is taken from the view the histograms were measured in.
  - Values (renders `screenshots/look_fixes/t5b_fix1/`, A HEAD / B; gain, L_a cd/m^2, veil): see the T5b report's "Fix
    round 1". Test `game/tests/glare_veil_test.gd` (headless): the form at 10 deg (1e-6); a Sun alone in a black frame
    (masked, seen) and the same Sun 20 deg off axis just outside a narrower frame give the same L_a (16 575 cd/m^2, 1e-6);
    across the frame edge (25.47 +- 0.1 deg) within 2 %; a hidden Sun (no seen sample), a Sun behind the eye, an
    enclosed view's out-of-frame Sun: no veil; the Sun at the axis finite; stars vanish 20 deg from the Sun (Schaefer
    limiting magnitude -5.3 at that L_a) and stay with the Sun behind (7.9). `adaptation_meter_test.gd`: the mask's
    parameters (image-centre ray, Sun along -Z, no Sun no mask) and the masked Sun's black frame field 0. Stereo mock
    (both eyes, Moon, Sun 12 deg above the view centre): mask 710 / seen 710, veil 8627 cd/m^2, gain 1.000.

- **Display tone step: chroma-preserving tone curve** [look fixes T5, 2026-11-07; `display_tone.gd` `DisplayTone` (model
  and CPU mirror), `display_tone_effect.gd` `DisplayToneEffect` (compute step); fidelity flag `DisplayTone.enabled`,
  render_check / cockpit_shots `--tone=agx`]. Godot's AgX (inset matrix, the allenwp curve per channel, outset) took
  16-26 % of the sky's and the ocean's B/R at mid-high scene values and turns the noon sky's OkLab hue by ~8 deg; a plain
  luminance curve keeps B/R but its per-channel clip turns saturated blues violet. The step, per pixel x (scene-linear
  Rec.709 after the post step; on every view layer, after `VisionEffect`, before glow and AgX): (1) the luminance Y goes
  through AgX's own grey curve, g = awp(Y) (Godot's contrast 1.25 and white 16.29 read from the environment) - the
  former tonality on the grey axis; (2) the chromaticity is kept, d0 = x g / Y: OkLab [S: Ottosson 2020] is homogeneous
  of degree 1/3, so d0 has x's OkLab hue exactly; (3) the Hunt factor's daylight part: OkLab chroma x max(S, 1) at
  constant lightness and hue; (4) path to white [call]: chroma x (1 - smoothstep) over log2 Y from scene 2 (above the
  sunlit white plate, 1.5) to AgX's white - Sun disc, glints and the brightest highlights end in display white; (5)
  gamut mapping at constant OkLab lightness and hue [call]: C_max(L, h) = the largest chroma AgX can show (in the sRGB
  cube and reached from a non-negative AgX input; 12 bisection steps, skipped while C / 0.75 is reachable), C / C_max
  compressed softly above the knee 0.75 (1 - 0.25 exp(-(r - 0.75) / 0.25)); (6) the pixel is written as AgX's exact
  inverse of the result, INSET^-1 awp^-1(OUTSET^-1 d), so the engine's AgX shows d (encoding unchanged; on the grey
  axis the pixel is untouched); at and above AgX's white the pixel is lifted along grey, x + max(0, white - min(INSET
  x)) (INSET maps grey to grey), to at least white in every AgX channel (white on screen, its largest channel at most
  doubled). Glow (fix round 1, review I1): Godot's glow reads this buffer before AgX with a largest-channel threshold
  (copy.glsl, world.gd 4.0), and P is steep near AgX's gamut edge (awp^-1(0.95) = 8.8): unbounded, the noon aureole
  (scene (1.75, 2.15, 2.35), max 2.35, no glow before) was written with a blue channel of 4.9-9.35 and glowed blue.
  The Hunt boost (3) is therefore gamut-mapped (5) into the colours whose largest written channel is at most max(the
  scene's largest channel, the glow threshold / glow strength) (`DisplayTone.glow_bound`, INF with glow off), but
  never below the chroma the step shows without the boost (unbounded): the boost starts no glow and strengthens none
  (written <= max(threshold, max x, the S = 1 value)); at S <= 1 the step is as without the bound, so display-referred
  emitters (their S = 1 value is their former c) keep their former glow. The aureole is then written at max 3.91
  (unbounded 4.90). A/B/C (pre-T5 / T5 / fix round 1, `screenshots/look_fixes/t5_fix1/`): pad noon Sun view, displayed
  luma of the aureole r 40-60 px 0.760 / 0.839 / 0.760, r 60-100 0.737 / 0.805 / 0.736, r 100-200 0.652 / 0.660 /
  0.651 (the T5 blue glow gone; the colour stays the scene's, B/R 1.07 / 1.38 / 1.35); night plume halo (97-99.5 %
  band) 0.144 / 0.153 / 0.153, noon plume 0.403 / 0.399 / 0.399; cabin (pilot eye, 97-99.5 %) 0.822 / 0.826 / 0.825.
  The night plume's ~6 % wider halo is the emitters' inverse under additive blending (T5), not the Hunt boost (S < 1
  there). Active only while
  the environment's tone mapper is AgX at `tonemap_exposure` 1 with SDR output (`DisplayServer.
  window_is_hdr_output_enabled` false: the inverse assumes AgX's output max 1; the linear camera exposures and the
  GPU checks' linear environments stay colourimetric). Display-referred emitters take the step's inverse (above)
  and look as before.
  Tests: `game/tests/display_tone_test.gd` (headless: grey written unchanged and shown on AgX's grey curve, AgX o step
  = target within 1e-5, monotonic in the scene level, OkLab hue kept within 0.5 deg over 3600 random colours at S 1
  and 1.64 (largest 0.0000 deg), the noon sky's B/R kept (AgX turns its hue 7.1 deg), Hunt = OkLab chroma x S at the
  same lightness, path to white, S's values / monotonic / cap / flag, cone_map's part <= 1, the emitters' inverse =
  AgX within 3 %, largest 0.7 %; glow bound: written <= max(4, scene, the S = 1 value) over 1500 random colours at S
  1.64, hue kept and AgX o step = target under it, the noon aureole 4.90 -> 3.91, emitters at S 1 unchanged by it,
  the grey lift display white with the largest channel <= 2x (1.18), u >= 0 on the sRGB cube);
  `game/tests/display_tone_check.gd` (GPU: 10 patches (with the noon aureole) through the shader at S 1 and 1.64, at
  S 1.64 with glow on (threshold 4, intensity 0) and with the step off against the CPU mirror, largest error 7e-4 of
  display-linear, tolerance 3e-3; 3 display-referred emitter patches = AgX, largest 1.9e-3, tolerance 1e-2).
  A/B (A = AgX + S against L_day, B = this step + S against the display; `screenshots/look_fixes/t5/`, report
  `.superpowers/sdd/2026-11-07-look-fixes/t5-report.md`): pad noon el 5 (S 1.58) sky el ~29 sRGB (82, 118, 154) B/R
  3.81 -> (62, 111, 182) B/R 9.85, OkLab hue -110.3 -> -102.4 deg = the scene's -102.4 (AgX turned it 7.9 deg); zenith
  (el 85, S 1.53) (105, 133, 163) -> (87, 128, 192), hue -109.0 -> -101.1 (scene -101.1); displayed luma within 7 %.
  Start view (weather analog, S 1.61): ocean (60, 81, 110) B/R 3.50 -> (61, 82, 125) 4.36 (EPIC 4.8-5.6); 400 km nadir
  ocean (48, 72, 102) -> (39, 68, 117) (ISS photo (24, 76, 123)). Sun disc unchanged (75 px at display white in both,
  centre (247, 247, 247) -> (245, 250, 254)); plume core noon (253, 233, 219) -> (255, 234, 221), night (254, 228, 202)
  -> (255, 231, 213). Cost (the step + its bisection only where C / 0.75 leaves the gamut): pad view 1600 x 900 median
  1.858 / 1.860 -> 1.923 / 1.905 ms (+0.06 ms), stereo mock 3534 x 2838 per eye 5.910 / 5.732 -> 6.041 / 6.034 ms
  (+0.13 / +0.30 ms, two rounds).

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
- Highlight cut of the lit weight (user 2026-11-07): `AdaptationMeter._field_bins` cuts HIGHLIGHT_SHARE (2 %) of the weight
  above black (bins >= 1), not of the whole field including the black bin; applies to the field mean and the anchor (rule
  D). Purpose kept: a Sun disc or glints of up to 2 % of the lit weight never set the field (`adaptation_meter_test`: a
  1 % Sun disc in a 90 % black field is cut; a 2.4 % lit area in a black field is the field; `adaptation_surround_test`:
  lunar el 60).
  Side effects (renders `screenshots/look_fixes/t5b/`): the visibility V keeps the old cut of the whole weight
  (`field_log_average(.., cut_of_whole)`), else a small lit object only the frame has (the Earth's disc in the lunar
  sky, absent from the model) closes V (first render: gain 12.99). A lone small lit object now enters the combined field
  (lunar day el 60: the Earth disc, near the view centre, field 2617 -> 4048 cd/m^2, gain 2.98 -> 1.93); the start view
  1.401 -> 1.269 (the cut of the ~30 % lit weight leaves more bright cloud in the field). The Sun's disc no longer
  depends on this cut: it is masked out by its direction and its glare is added as a veil ("Sun glare" above).

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
- Adopted white = the field's illuminant, estimated physically (Sun + sky, moonlight, the enclosure's lamps, the Sun
  above the air in space; not a grey world of the frame), with CIECAM02's D (average surround, F 1), paced 26 s (user
  2026-11-08, after review T5b Q3; it replaces the fixed TOA-Sun white of proposal F7, 2026-11-07).
- The Sun's disc is masked out of the field meter by its direction and size, its disability glare added as a veil
  (Stiles-Holladay, CIE 146:2002) - in the frame and just outside it, both eyes; no solid-angle floor (user 2026-11-08).
- Hunt factor against the display's white (100 cd/m^2), no cap at 1 by day, and the chroma-preserving display tone
  step on AgX's grey curve, hue-stable in OkLab (look fixes plan, user 2026-11-07 "Alles wie vorgeschlagen umsetzen",
  proposals F2 + F3).

## Open points and known limits

- Display tone step (T5): with the chroma kept (and x1.6 by day) the clouds' residual warmth after the white balance
  shows - cloud radiance B/R 0.93-0.95 of the TOA Sun's white (start view / 400 km), displayed B/R 0.88-0.92 against
  AgX's 0.97-0.98 and EPIC's 1.00-1.04: the clouds read pink. Candidates: the adopted white (the eye adapting to the
  scene's bright field, the clouds, rather than the TOA Sun) or the clouds' TOA colour (T7; review: a missing cloud /
  air coupling). The lunar regolith's teal is fixed (T5 fix round 1: `moon_sun_color`) and its red slope set to the
  measured colour (T5b: `moon_albedo_tint`, moon-surface.md). The S reference moved civil dusk from S 0.68 to ~1.0
  (more colourful twilight) - user's verdict open. Glow: with the bound, emitters at S > 1 (the plume by day) glow
  somewhat less than with AgX alone (their S = 1 value is their former c through the Hunt inverse, less saturated).

- Adopted white: the pad's global horizontal daylight (Sun 48 deg) is B/R 0.758 / 5510 K against the TOA Sun's 0.856
  / 5920 K, so by day the ground reads more neutral and the sky bluer than with the former fixed white (pad noon el 5:
  ground B/R 0.55 (TOA white) -> 0.68, sky 7.5 -> 10.3; el 60 sky 9.5); at dusk (Sun -5.5 deg) the twilight sky light
  (B/R 2.9) at D 0.83 turns the blue hour nearly grey (sky B/R 3.1 -> 1.25, the horizon glow orange) - D at mesopic
  levels is a pending user decision. The full
  Moon at a VR fov stays display white (B/R 1.00 in A and B): adapted to the moonlit night (L_a 0.04 cd/m^2, gain 1.2e4)
  the disc is far above white - the Moon's own disability glare (10 x 0.3 lux / 0.27^2 ~ 40 cd/m^2 looking at it) and
  local adaptation are not modelled; pale cream shows where the disc is in range (fov 2: B/R 0.33 -> 0.77). The surround
  model has no moonlit sky (black at night), so at night the fov-2 frame still stands alone (out-of-frame share 0).
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
  on the Moon the surround brightens the field. Since 2026-11-07 the 2 % cut is of the lit weight only, so a lit surround that is a
  small share of a mostly black field (lunar day 60 deg up: 2.4 % of the weight) is no longer cut whole.
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
