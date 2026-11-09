# Eye adaptation, night vision and Moon light (M8)

History and the reasons behind superseded decisions: `docs/history/atmosphere-rendering.md`,
`docs/history/eye-adaptation.md` (fix-round accounts, A/B numbers).

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
- **Unified appearance model - the game's rule** [look fixes T13, user 2026-11-08: one observer model in the spirit
  of iCAM06 (Kuang, Johnson & Fairchild 2007) / Pattanaik et al. 2000's multiscale observer, replacing the special
  cases; `FieldRule.MEAN_GREY`, default]. (1) Global temporal adaptation: the adopted white L_w = L_f / Y_b,
  Y_b = `FIELD_MID_GREY` = 0.20 [S: CIE 159:2004 CIECAM02, the background of an average scene, 20 % of the white],
  capped at the white plate under the TOA Sun (`EyeAdaptation.advance`; no diffuse surface is whiter: a field brighter
  than Y_b of it is a white world shown at the white - the former "gain floor 1"); Ferwerda gains, pacing, mesopic k,
  CIE 191 D, Hunt S and Stevens B unchanged. The adopted white renders at the scene units' white plate (DAY_EXPOSURE),
  the field at Y_b x DAY_EXPOSURE = 0.3. (2) Local adaptation (below) holds every bright region (cabin walls and
  windows, the Earth from the lunar night, the full Moon) at the anchored white - the job rule D's anchor term did
  globally. (3) Chromatic adaptation and colour appearance (CAT02, mesopic D, Hunt, Stevens; below). (4) Physical glare
  spread (CIE 146) in place of Godot's glow (below). (5) The fixed display mapping (`DisplayTone.display_curve`, display
  tone step 1, below): linear (colourimetric) to the knee - the field at 0.2 of display white - and a log-quadratic
  shoulder to display white at DAY_EXPOSURE x 1000 / 203 [S: ITU-R BT.2408: HDR reference white 203 cd/m^2 on a
  1000 cd/m^2 peak]: white plate 0.67, scene 2.5 0.83. Removed (history `docs/history/atmosphere-rendering.md`): rule D
  as default (`--protect_adaptation`), the T6 rule key / `--day_exposure` (`--field_key=0.12` gives its key), the
  cockpit hand keys (`cockpit_shots --cabin_keys`), the planned T11 Moon exception and T12 highlight shoulder, AgX's
  grey curve in the tone step (`--display_curve=agx`), Godot's glow (`--glow=on`). Reference scene suite (the
  definition of done for every later appearance change): `tools/look_reference_suite.py` (scenes, targets and sources
  below in "Reference scenes"). Test: `adaptation_meter_test.gd` (Y_b, default rule, field at Y_b x DAY_EXPOSURE and
  at 0.2 of display white, white cap), `display_tone_test.gd` (T13 cases).
- **Rule D (option `--protect_adaptation`)**: L_a = max(L_f / `field_key`, L_anchor / `ANCHOR_WHITE`), the stricter
  (darker) of the mean-grey and the anchor exposures (the game's rule 2026-11-07 .. T13). Comparison A field /
  C anchored / D protected: `screenshots/eye_adaptation/protected/`.
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
  - Limits (current): the global adaptation is one value per view (both VR eyes); bright regions adapt locally (T10,
    `LocalAdaptation`, gaze-free, no foveal weighting) but a structure narrower than ~2 blocks stays white; disability
    glare is the CIE 146 veil of every source (Sun, Moon, Earth: `GlareSpread`, shown at its source's local factor, the
    far veil > ~10 deg below the field is invisible); no loss of acuity; the cones' degree of adaptation D is mesopic
    (CIE 191), the Moon's field tinted toward the regolith; the rods' slower dark adaptation (~30 min) is paced like the
    cones' (110 s); no airglow, zodiacal light or integrated starlight (a moonless sky is black); Jensen's k on the
    adaptation luminance, not per pixel; B and S one value per view, no Bezold-Bruecke hue shift (lunar day keeps
    B = 1, S 1.66). See "Open points and known limits".

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
    Near the Moon (bug hunt 2026-11-08, user-approved; `Atmosphere.moon_field_tint`, flag
    `Atmosphere.moon_field_adaptation`): the eye's field there is regolith-reflected light against a black sky, so both
    plates move toward the regolith's colour (`moon_grey_tint`) by `MOON_FIELD_MEAN_WEIGHT` 0.6 [call: mixed / partial
    adaptation, Katoh & Nakabayashi 1997, CIE 162:2004; Helson-Judd effect] x the Moon's share of the surroundings
    1 - cos(theta) (1 on the ground, 0.48 at 300 km, 0.23 at 1000 km, ~1e-5 from the Earth: the full Moon from the Earth
    is unchanged). Ground tint B/R 0.786; lunar day ground (suite `lunar_day_apollo11`) displayed B/R 0.638 -> 0.922, full Moon from the Earth 0.788 unchanged; see the
    bug-moon report, gate 0.75-0.95 [Apollo crews: grey, brownish-grey at low Sun]. Test `moon_colour_test.gd`.
    Illuminant B/R and CCT [linear Rec.709, McCamy]: pad Sun 48 deg 0.803 / 5706 K since T9b (direct 5135 K, sky 10 726 K,
    sky 17.3 %; white_balance_test, maritime weight of the pad 0.55); before T9b (the values below): 90 deg 0.784 /
    5620 K, 60 deg 0.772 / 5570 K, 48 deg 0.758 / 5510 K (was 0.724 / 5370 K), 30 deg 0.718, 15 deg 0.654, 5 deg 0.753 (sky-dominated; was 0.181), 0 deg
    1.52, dusk -5.5 deg 2.93 (unchanged, sky only); 400 km and the Moon 0.856 / 5920 K (the TOA Sun, unchanged).
    Reference: D55 = B/R 0.745, D60 0.874, D65 1.0 in linear Rec.709; measured clear-sky global daylight at high Sun
    ~5500-6500 K - the model sits in the band's lower half (before T9b at its warm edge: the 680 nm red channel). Known limit: an
    eye high in the air with the Sun below its local horizontal but still visible (the horizon dip) takes the sky's
    colour only (the scene below it is in twilight).
  - Degree of adaptation [S: CIE 159:2004; Moroney et al. 2002]: D = F (1 - (1 / 3.6) exp((-L_A - 42) / 92)), L_A = the
    paced L_a x Y_b / 100 (Y_b 20), F = `SURROUND_F` 1.0: the scene observer's surround is the scene itself, CIECAM02's
    "average" (S_R >= 0.2; dim 0.9 and dark 0.8 are films / slides in a dim / dark room) [S: CIE 159:2004 Table 1;
    Fairchild 2013, Color Appearance Models 3rd ed., 16.4]; the display observer fully adapted to the display white
    [call]. This photopic D (`photopic_degree_of_adaptation`) is 0.90 in the lamp-lit cabin (L_a ~270), 1.000 by day
    and in space; its floor 0.82 F at L_A -> 0 is an extrapolation of photopic data.
  - Mesopic degree of adaptation [T5b fix round 3, user 2026-11-08 "the blue hour must stay blue";
    `degree_of_adaptation(L_a, L_as)`, `mesopic_photopic_weight`]: D = m D_photopic, m the photopic share of CIE 191's
    mesopic response m V + (1 - m) V' [S: CIE 191:2010: iterate L_mes = (m L_p + (1 - m) L_s V'(lambda_0)) / (m +
    (1 - m) V'(lambda_0)), m = 0.7670 + 0.3334 log10 L_mes in [0, 1], V'(lambda_0) = 683 / 1699], evaluated at the
    adapting field L_p = L_A = 0.2 L_a and L_s = 0.2 L_as (the paced scotopic adaptation: the field's S/P). [call] The
    von Kries gains are a cone mechanism; in the mesopic range only the cone part m of the response discounts the
    illuminant's colour, the rods have no cone-specific gain control - their achromatic signal and blue tint stay the
    rods' post step (Ferwerda k / Jensen, unchanged; m and k are separate quantities: k weighs the rods' luminance, m
    the cones' white point - no double handling). The CIE's explicit photopic / scotopic switches are left out (with
    the clip they are the iteration's fixed points for S/P >= 1; for S/P < 1 the switch at L_p 5 would jump), so D is
    continuous in L_a. Values: m = 1 for L_A >= 5.06 cd/m^2 (L_a >= 25.3: day, cabin, the lit Moon, space - D
    unchanged, 1e-12), m = 0 for L_mes <= 0.005 (moonless night: the cones keep the colourimetric light, below the rods
    anyway). D at S/P 3 (twilight sky): L_a 10 0.730, 3.16 0.605, 1 0.484, 0.1 0.246.
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
  Test `white_balance_test.gd` (below).
  Why a white balance at all (T2): the renderer's colours are colourimetric linear sRGB (white D65), but the Sun above
  the air is warmer: the core's TOA spectrum in linear sRGB is (1.123, 1, 0.961), B/R 0.856. Without adaptation a sunlit
  white cloud seen from space was drawn salmon (start view: cloud radiance B/R 0.79-0.84, physical; displayed B/R
  0.95-0.96 against EPIC's neutral). With the eye adaptation off (fixed exposure, `--camera_exposure`) the image stays
  colourimetric. Cost: the post step runs every frame: pad view 1600 x 900 not measurable, stereo mock 3534 x 2838 per
  eye +0.11 / +0.19 ms (T2). Test `game/tests/white_balance_test.gd` (headless): the D65 grey kept (identity), the TOA
  Sun B/R 0.856 drawn grey at its luminance (1e-6), the Sun through Rayleigh air mass 4 at 700 hPa still warm at D = 1
  toward the TOA Sun (B/R 0.485 -> 0.564), post step o display-referred inverse = identity (1e-6, also at D 0.82 toward
  the moonlight); photopic D = 0.8240 at L_A 0, 0.9022 at a 270 cd/m^2 white, 1 by day; CIE 191 m = 0.7670 + 0.3334
  log10 L at S/P 1 (1e-9; 0 at 0.004, 1 at 5.1), its fixed point at S/P 3 (1e-9); the mesopic D = the photopic for
  whites >= 25.3 cd/m^2 at S/P 0.5-3.5 (1e-12), continuous (largest step < 0.005 per 0.01 dex over 1e-4 .. 1e5) and
  increasing, below the photopic at dusk by > 0.05; D 0 the identity, gains
  linear in D; the full Moon's light (Sun x regolith colour x air at el 83) adapted with the photopic floor D 0.824:
  B/R 0.874 (pale cream, band 0.80-0.92; D 1: 1.000; the former TOA white 0.505; colourimetric 0.436), with the
  night's mesopic D between colourimetric and that cream; the moonlight colour keeps the
  Sun's luminance (1e-6); a D65 cabin lamp neutral (1e-6); pacing one time constant e^-1 (1e-9), 90 % in 60 s (0.0995),
  a snap jumps; the field illuminant at the pad at noon (Sun 48 deg) = the horizontal Sun + sky (1e-6) with its CCT in
  5200-6500 K (5706 K; [S] Hernandez-Andres et al. 2001 clear-sky global daylight, lower edge [call] -300 K for the
  model's spectrum). T2's A/B numbers: `.superpowers/sdd/2026-11-07-look-fixes/t1-t2-report.md`.

  Under broken clouds (look fixes T13, 2026-11-08) the illuminant's colour takes the weather grid's MEAN cloud light at
  the eye (`cloud_light_at` without the shaped "eye" probe): standing in a cumulus' shadow the plate had the sky alone
  (B/R 1.44, D 1) and the sunlit clouds in view went orange (pad noon el 60 cloud B/R 0.61); now B/R 0.74 as without
  weather and the cumulus white (`screenshots/look_fixes/t13/suite/C_pad_noon_e60.png`). The luminance terms keep the
  shaped probe.
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

- **Local adaptation: small bright objects keep their detail** [look fixes T10, user 2026-11-08; `local_adaptation.gd`
  `LocalAdaptation` (model, CPU mirror), `local_adaptation_effect.gd` `LocalAdaptationEffect` (two compositor stages);
  fidelity flag `LocalAdaptation.enabled`, user argument `--local_adaptation=off` (game, render_check, cockpit_shots)].
  The global rule sets one gain for the view; a small object far brighter than the field - the Earth from the lunar
  night (~1e4 cd/m^2 at gain 7557: scene ~1000, display white with Godot's glow around it), the full Moon in the Earth's
  night sky, lamps in a dark cabin - clipped. The eye adapts locally [S: Vangorp, Myszkowski, Graf & Mantiuk 2015, "A
  model of local adaptation", ACM TOG 34(6) 166: the adapting luminance is pooled over a Gaussian of sigma 0.428 deg
  (Eq. 10; the adapting field levels off at a ~0.5 deg disc, rings beyond ~6 deg have no effect)], and light
  adaptation is fast while dark adaptation is slow (Pattanaik et al. 2000), so local adaptation only lowers the gain:
  g_px = min(g, G(L_eff)), G the global rule's cone gain (`EyeAdaptation.cone_gain`, Ferwerda, [1, 1e5]), L_eff = L_loc
  x DAY_EXPOSURE / (PLATEAU d^2), L_loc the pooled local luminance. In the Weber range a locally adapted region shows its
  local mean at the scene value PLATEAU = `EyeAdaptation.ANCHOR_WHITE` x DAY_EXPOSURE = 1.35 [E: anchoring, Gilchrist
  et al. 1999 as in the anchored rule - the bright region is the anchor of its neighbourhood, seen as white of
  reflectance 0.9 under the TOA Sun; sRGB ~213, below AgX's shoulder, so the detail stays]; the displayed local mean
  rises with L_loc up to PLATEAU, stays there and rises again where G reaches its floor 1 (above the white plate under
  the TOA Sun: the Sun's disc stays display white) - monotonic, no inversion. A region whose global display value is at
  most PLATEAU is untouched (pad noon, lunar day, the Earth from orbit by day: gain 1, nothing to lower). The rods'
  part of the post step (VisionEffect, 38 % of the Earth's displayed Y at lunar night) is scaled by c^2 (the cones' by
  c): rod saturation [E: Aguilar & Stiles 1954] - a region the cones adapt to locally is photopic and has no rod signal;
  the shape c^2 is a [call] (only "the rods' share vanishes under strong compression, continuous at c = 1" is justified).
  Pooling (look fixes T13, grid-phase invariant; T10's block grid varied a 0.3-1 deg disc by x1.1-1.9 with its phase):
  L_loc is a shift-invariant edge-aware pool, Vangorp's Gaussian made edge-aware as a piecewise-linear bilateral filter
  [S: Durand & Dorsey 2002; Paris & Durand 2006]: 16 log-luminance bands (spacing <= 0.4 decades [S: sigma_r], from
  PLATEAU/16 to the fp16 clamp [call]; the lowest band takes every darker pixel); per band the range weight of every
  full-resolution pixel (sigma 0.4 decades for a neighbour at or below the band, 1.0 decade for a brighter one [call:
  the antialiased edge pixels of a bright disc share its factor instead of staying white]); MAP: area-exact box
  downsample of w L and w to a block grid (block = largest power of two with >= 1.5 texels per sigma, <= 32 [call];
  8 px at the headset's 31.5 px/deg; every pixel read), separable Gaussian blur at low resolution (radius 3 sigma
  [call]); APPLY: bilinear upsampling, each pixel reads its own two bands (linear in its log L): L_loc = interpolated
  blur(wL) / blur(w). Box x Gaussian is shift invariant (aliasing exp(-2 pi^2 (sigma / texel)^2) < 1e-19 at 1.5 texels).
  A pixel whose like-pixel share blur(w) is below 1 % keeps c = 1 (an emitter over a dark field, a sub-block glint), a
  smoothstep blends log c in up to 4 % [call]. The edge-aware pooling is a display requirement, not the eye's: Vangorp's
  plain Gaussian darkens a ring of the dark field around a bright disc and brightens its limb; here the dark field
  keeps c = 1 and the limb no more than the interior (test). MAP runs before the transparent pass (after the field meter,
  with the cloud composite; MSAA resolve every frame), APPLY after VisionEffect, before the display tone step. Per view
  (both VR eyes) with ONE block size per frame from view 0's px/deg at the image centre [call] (wider in degrees at the
  periphery of a wide frustum, accepted). No temporal filter: a screen-space low-pass cannot follow a moving object (a
  30 deg/s head turn at the 0.5 s light constant would drag a ~15 deg ghost); fast neural light adaptation (~0.1 s,
  Pattanaik 2000) is faster than a frame at these scales, so c follows the same frame. Measured (CPU mirror, 31.5
  px/deg, a 1e4 cd/m^2 antialiased disc over 1e-3 moved 0..8 px in half-pixel steps): 9.5 / 16 / 24 / 32 px (0.3 .. 1
  deg) displayed flux x1.02 / 1.02 / 1.01 / 1.01, peak x1.03 / 1.02 / 1.01 / 1.01 (gate 5 %; T10: x1.86 .. 1.12);
  stereo through the GPU path (disparity 0-8 px) left / right peak within x1.005 (gate 3 %); GPU = CPU within 0.7 %
  (fp16 maps). Below ~0.3 deg (6 px) the share blend keeps a phase-dependent factor (flux x2.7 over a block,
  information): the thin Earth crescent and the far Moon on the desktop (1-4 px) stay c = 1. One source of truth: the
  shaders are templates filled from `EyeAdaptation` / `LocalAdaptation` (`LocalAdaptation.band_count()`). Off with the
  eye adaptation off, at a display gain <= 1 and before the tables have loaded. Cost at 5294 x 4250 per eye stereo
  (RTX 4090, `--gpu_passes`, median of 300 frames, GPU shared with other agents): MAP 0.73 ms + APPLY 0.44 ms (T10's
  block grid 0.27 + 0.33 ms), plus the MSAA resolve before the transparent pass. In-game (lunar night, A T10 grid / B
  bands; `screenshots/look_fixes/t13/la/`): Earth at fov 100 (12 px) 50.8 % -> 8.5 % of the disc clipped; full Moon fov
  12 17 % -> 0 % clipped, no glow rim; crescent identical (sub-block).
  Local chromatic adaptation (T13, iCAM06-style [S: Kuang, Johnson & Fairchild 2007: the chromatic adaptation's white
  and degree are local]): a locally adapted region is photopic, so its cones take the white balance at D = 1 to the
  same adopted illuminant: cones' = mix(cones, K cones, 1 - c), K = the cones' map at D = 1 over the frame's
  (`LocalAdaptation.local_white`, `apply_pixel`) [call: the weight 1 - c]. The full Moon over the moonlit night
  (illuminant moonlight, global D 0.15-0.73) goes from cream to a warm off-white (disc centre sRGB (237, 232, 220));
  the Earth from the lunar night (illuminant earthshine in the Sun's colour) keeps its blue. Test `local_adaptation_test`
  (a white under a warm illuminant at D 0.15, locally adapted: neutral within 1 %).
  - The halo (the user's "blur rim", 2026-11-08): Godot's glow (world.gd: threshold 4, intensity 0.6, its input capped
    at luminance 12 - independent of the source's luminance and of the eye), not the glare veil (that is the Sun's alone
    and enters only the adaptation) nor the Earth shader: with `render_check --glow=off` the rim is gone (A, lunar night
    under the Earth, luma at 3 / 6 deg from the disc's centre 172 / 55 with glow, 0 / 0 without). For comparison the
    CIE 146:2002 disability glare of that Earth (E 6.9 lux at the eye, young eye, p 0.5) displayed at the view's gain
    would be sRGB 223 / 186 / 103 / 57 at 2 / 3 / 6 / 10 deg against the glow's 213 / 172 / 55 / 8: the glow is of the
    veil's order out to ~5 deg but falls off faster, and it is not tied to it. With local adaptation the disc is written
    below the glow threshold, so Godot's glow no longer fires on it. Since T13 Godot's glow is off and the veil of every
    source is drawn physically ("Glare spread" below).
  - Values (A `--local_adaptation=off` / B; sheets and crops `screenshots/look_fixes/t10/`, all numbers in the report
    `.superpowers/sdd/2026-11-07-look-fixes/t10-report.md`): lunar night 0 N 45 E under a ~97 % Earth (gain 7557) disc
    sRGB (254, 254, 254), 96 % clipped -> (188, 209, 244), luma sd 3 -> 18 (ocean, Africa, clouds); gibbous / half Earth
    likewise (half disc kept); full Moon over Starbase at fov 12 (gain 6645) (255, 255, 255) -> (233, 211, 189) with the
    maria, also in the stereo mock; Earth from space (gain 1) pixel-identical; pad noon el 5 and Apollo 11 lunar day el
    5: run noise only (0.04 % / 0.1 % of the pixels differ by > 2, both signs); cabin pilot_eye (gain 157): the daylit
    windows show the sky (saturated pixels 1.9 % -> 0.08 %), walls and screens unchanged. Cost (RTX 4090, stereo mock at
    the headset's 5294 x 4250 per eye, GPU timestamps, median of 300 frames, two rounds, GPU shared with other renders):
    MAP 0.27 ms, APPLY 0.33 ms (pixels with no compression near them are not read), plus the MSAA resolve before the
    transparent pass every frame (0.26 ms at 3534 x 2838, Task 14); first version 0.60 / 0.46 ms.
  Test: `game/tests/local_adaptation_test.gd` (headless, CPU mirror): pad noon's sky untouched; a bright region at the
  plateau (1e-9); the displayed local mean monotonic over 1e-3 .. 1e10 cd/m^2; the Sun at the gain floor; no change at
  fixed exposure; the block size; the rods' part (c = 1 leaves the pixel, strong compression leaves c x the cones'
  image); an image of the Earth from the lunar night (2 deg disc of 1e4 cd/m^2 with +-40 % detail, blocks of 2 and of
  the step's own 4 px at 20 px/deg): its displayed mean 1.349 / 1.368 (tolerance 25 %), detail ratios within 0.3 / 0.7 %
  (5 %), the 864 ground / sky pixels beside it untouched, limb max 1.908 / 2.034 against the interior's 1.904 / 1.962
  (1.25x), an emitter over the black sky c = 1; translation invariance (disc of 10 .. 32 px over the block grid, bound x2);
  a soft 1/r^2 gradient (factor rises outward, no ring); an image size that is no multiple of the block (disc at the edge);
  the shaders carry the generated constants (no placeholder left); the flag. `game/tests/local_adaptation_gpu_check.gd`
  (needs a GPU, local RenderingDevice, no render): the generated shaders against the CPU mirror on 12 discs of 9.5 .. 32
  px (worst |c_gpu / c_cpu - 1| 0.0068, tolerance 3 %, fp16 input) and the stereo disparity sweep (x1.005, gate 1.03).

- **Glare spread: the eye's own scatter, physical** [look fixes T13, 2026-11-08; `glare_spread.gd` `GlareSpread` (model,
  CPU mirror), `glare_spread_kernel.gd` (generated by `tools/glare_kernel_fit.py`), `glare_spread_effect.gd`
  (two compositor stages); switches `--glare_spread=off`, `--glow=on` (Godot's former glow, threshold 4, intensity 0.6;
  default off)]. The veil is CIE 146:2002's general disability glare function [S], L_veil / E_gl = 10 / theta^3 +
  (5 / theta^2 + 0.1 p / theta)(1 + (A / 62.5)^4) + 0.0025 p (theta deg), A 25 [call: young observer], p 0.5 [S: CIE
  146 average pigmentation], as 10 non-negative Gaussians sigma 0.1 deg x 2^k (NNLS on the relative error; <= 3 % over
  0.5 .. 30 deg, +18 % at 0.2, 46 % at 0.1 deg (the core); energy over 0.1 .. 60 deg -3.6 %). Sources: only light the
  display cannot show [call: the viewer's own eye scatters the displayed light], max(0, Y - DAY_EXPOSURE) per pixel
  (colour kept, solid angle per pixel); the Sun's disc is masked (the meter's mask) and its veil drawn analytically per
  pixel (CIE f x normal illuminance x cos x visibility, in and out of frame). SPREAD stage post-transparent before
  VisionEffect (tent-filtered pyramid, a Gaussian per level, coarse-to-fine; frames under 15 px/deg merge the sub-texel
  levels into level 0); ADD stage after the local adaptation's APPLY, through VisionEffect's map (rods and cones see the
  veil). **Shown at its source's local factor** [T13 fix 1, user / orchestrator 2026-11-08; `GlareSpread.source_factor`]:
  the veil is light on the retina tied to its source, and the eye adapted to the full Moon / the Earth / a lamp sees the
  halo relative to that source - source and halo are one framework [E: Gilchrist et al. 1999 anchoring, the local
  adaptation's basis]. So every source pixel's veil is shown at that pixel's local factor c = clamp(PLATEAU / Y, 1 / g,
  1) [call: the local adaptation's Weber-range form, floored at the cone gain's floor 1 / g] - the SOURCE stage spreads excess x c in the pyramid's alpha, the ADD stage scales the veil by alpha /
  Y(veil). The halo thus keeps its physical ratio to its disc (CIE 146 x Omega: full Moon 9.7e-4 / 1.6e-4 / 1.9e-5 at
  1 / 2 / 5 deg, full Earth from the Moon 2.2e-3 / 2.6e-4 at 2 / 5 deg), monotonic, no limb logic. The **Sun's analytic veil**
  is shown the same way [user 2026-11-08 "Sonne wie Mond und Erde"; `Atmosphere.sun_disc_scene`]: at its disc's factor
  PLATEAU / Y_disc (Y_disc = beam illuminance / 6.8e-5 sr x scene per cd/m^2, ~7e6 x g; no floor - the disc is clamped to
  white, its halo is not), so the halo is CIE 146 x Omega_Sun of the disc: 1.0e-3 / 1.7e-4 at 1 / 2 deg = sRGB luma 3.1 / 0.5
  (measured, Sun in space 400 km, 60 and 10 deg; before: white out to 3 deg, 211 at 5 deg, a "Godot blur"). From the ground
  the sky shader's aureole (unchanged) is the halo (luma ~225 at 1-3 deg); the disc is 51 px at 60 deg, not the clipped
  halo's 5600. The spread's Sun mask direction is the Sun seen from the CAMERA (`sun_apparent - camera`, not from the Earth's centre: from lunar orbit the 0.15 deg parallax left the disc's rim outside the mask, and the rim spread as a source - a 1.4 deg white ball). The meter still takes the full veil (disability glare raises the adaptation). Measured alternatives (`screenshots/look_fixes/t13_fix1/`):
  T13's veil at the global gain showed the full Moon's halo at 0.10-0.47 of the disc at 1 deg (a glowing ball; dark
  limb dots and axis-aligned steps from the per-texel limb compression); the veil added before the local adaptation's
  MAP (pooled with the image) was flattened to the plateau - a dark ring at the limb, a bright ring at 1-2 deg (halo_rise
  19-76 luma). The adaptation meter
  adds the last frame's mean image veil to the field and the anchor like the Sun's scalar veil (per sample it lowered
  the log average under a black sky). Measured (`screenshots/look_fixes/t13/glare/`): the lunar-night Earth's pyramid
  veil against the direct CIE sum -25 % at 1.5 deg, -6 .. 0 % at 2 .. 10 deg, radial profile strictly falling; a moving
  0.5 deg disc varies the veil at 2 / 5 deg by <= 0.9 % (gate 3 %); GPU = CPU within 0.0013 of the peak. The Sun in
  space (black sky, no aureole): luma at 1 / 2 / 3 / 5 deg 3.1 / 0.5 / 0.1 / 0 from 400 km (the veil at the disc's
  factor; before the Sun fix 255 / 255 / 251 / 211). Cost (stereo mock at the headset's resolution, shared GPU): SPREAD 1.10 + ADD 0.72 ms (frame +1.9 ms). Tests
  `glare_spread_test.gd` (headless, slow), `glare_spread_gpu_check.gd` (GPU). The scalar Sun veil of the adaptation
  (Stiles-Holladay 10 / theta^2) is unchanged (CIE 146 is 0.68-0.86 x it at 3-25 deg; open point).
  Stereo: each eye's frustum and orientation in the head come from its render projection (RQ decomposition,
  `GlareSpreadEffect.eye_frame` / `view_frame`), canted displays included (final-review fix a93d4938; test
  `glare_spread_test.gd` `_canted`: Sun centre within 0.000 px at +-10 deg cant; the Pimax Dream Air views are parallel).
- **Reference scenes (definition of done for every appearance change)** [look fixes T13; `tools/look_reference_suite.py`,
  needs a GPU, ~2 h for all 34 scenes; `--only`, `--label`, `--extra`, `--compare`]: pad noon el 5 / 60, dusk -5.5 and
  blue hour -7.7, lunar day (Apollo 11 site), lunar night with full / half / crescent Earth, the full Moon from
  Starbase's night, the Earth from space against DSCOVR EPIC 2025-03-20 14:28, the Sun in view (ground, 400 km, lunar
  orbit), the cabin (pilot_eye, day); the small bright objects also at 22.2 deg (40 px/deg, the headset's scale), 60 and
  10 deg (central vision) with crops; the terminator (half Earth 10 deg, 400 km) measured only until T9b. Every scene
  with the Earth or its sky in view runs with `--weather=analog` (the game's weather). Gates and sources in the script;
  results `screenshots/look_fixes/t13/suite/<label>_results.json` (`--out` for another folder). Halo profile (T13 fix 1,
  full Moon from the Earth and full Earth from the lunar night, every fov): linear display luminance of rings at 1 / 2 /
  5 / 10 deg over the disc's (`halo_<d>deg_rel`) against CIE 146 x the disc's solid angle (Earth 2 deg 0.001-0.006, 5
  deg <= 1e-3, 10 deg <= 3e-4; Moon 1 and 2 deg <= 0.03 incl. the sky / aureole floor) and `halo_rise` (8-bit radial
  profile from 1.15 radii falls within 3 luma: no limb / plateau rings). Horizon clouds at noon from 2 km
  (`horizon_clouds_noon_2km`, bug hunt 2026-11-08, user: red-shifted horizon clouds): displayed linear B/R of the far
  cloud band >= 0.75 and far / near cumulus >= 0.85 [D: the core's composite of a white cloud 30 km away, 0.845 of the
  plate white, shown at S 1.5 -> 0.775]; FAILS on 2026-11-08 (0.64 / 0.755) - open, cause and options in
  `known-issues.md` (Horizon clouds at noon). Gap: no night cabin with lamps (cockpit_shots has no night case).
- **Display tone step: chroma-preserving tone curve** [look fixes T5, 2026-11-07; `display_tone.gd` `DisplayTone` (model
  and CPU mirror), `display_tone_effect.gd` `DisplayToneEffect` (compute step); fidelity flag `DisplayTone.enabled`,
  render_check / cockpit_shots `--tone=agx`]. Godot's AgX (inset matrix, the allenwp curve per channel, outset) took
  16-26 % of the sky's and the ocean's B/R at mid-high scene values and turns the noon sky's OkLab hue by ~8 deg; a plain
  luminance curve keeps B/R but its per-channel clip turns saturated blues violet. The step, per pixel x (scene-linear
  Rec.709 after the post step; on every view layer, after `VisionEffect`, before glow and AgX): (1) the luminance Y goes
  through the fixed display curve g = `DisplayTone.display_curve(Y)` (look fixes T13: linear Y / DAY_EXPOSURE to the knee
  0.203, log-quadratic shoulder ln g = ln g_k + u - u^2 / 2U, u = ln(Y / Y_k), to white at DAY_EXPOSURE x 1000 / 203
  [S: ITU-R BT.2408]; `--display_curve=agx`: AgX's own grey curve awp(Y) as T5 .. T13, Godot's contrast 1.25 and white
  16.29); the emitters' inverse uses the same curve (global `vision_display_curve`); (2) the chromaticity is kept, d0 = x g / Y: OkLab [S: Ottosson 2020] is homogeneous
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
  (unbounded 4.90). Active only while
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

- Eye model: Ferwerda 1996 cone/rod t.v.i. with Jensen 2000 rod saturation, CIECAM02 brightness / chroma terms,
  Schaefer 1990 star threshold (user decision 2026-10-25, "like the eye").
- One appearance model (user 2026-11-08, look fixes T13): global mean-grey adaptation (Y_b 0.2, white cap), local
  adaptation for bright regions, CAT02 / CIECAM02 appearance, CIE 146 glare, a fixed display curve (BT.2408 headroom);
  no per-scene rules or keys. Every appearance change passes `tools/look_reference_suite.py`. (Rule D, the darker of the
  mean-grey and anchor exposures, 2026-11-07 .. T13: history.)
- Local adaptation for small bright objects (user 2026-11-08, look fixes T10): only ever lowers the gain, the local
  mean of a bright region at the anchored white (1.35), edge-aware pooling (no halo), switchable
  (`--local_adaptation=off`).
- Scene-referred adaptation to the measured field (user 2026-11-05, Task 14): one rule for every scene, one value for
  both VR eyes (formerly view-independent; the white-plate rule is the fallback). Earth-in-view stays physical (no
  display-side highlight treatment). Display-referred emitters never set the adaptation (the meter runs before the
  transparent pass; Task 14 fix round 1, review I-1).
- The field is wider than the frame (look fixes plan, user 2026-11-07 "Alles wie vorgeschlagen umsetzen", proposal F4):
  70 deg about the view axis from the analytic surround model, gated by its agreement with the frame (enclosed views
  keep the frame alone). Rule D, the highlight cut and the anchor percentile unchanged.
- City lights scene-referred with scale [E] (user ruling 2026-10-25).
- Adopted white = the field's illuminant, estimated physically (Sun + sky, moonlight, the enclosure's lamps, the Sun
  above the air in space; not a grey world of the frame), with CIECAM02's D (average surround, F 1) x CIE 191's mesopic
  cone share m (fix round 3, user 2026-11-08: the blue hour stays blue), paced 26 s (user
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

- Adopted white: the pad's global horizontal daylight (Sun 48 deg) is B/R 0.803 / 5706 K (T9b; before 0.758 / 5510 K) against the TOA Sun's 0.856
  / 5920 K, so by day the ground reads more neutral and the sky bluer than with the former fixed white (pad noon el 5:
  ground B/R 0.55 (TOA white) -> 0.68, sky 7.5 -> 10.3; el 60 sky 9.5); at dusk the twilight sky light (B/R 2.9) is
  the adopted white, with the mesopic D (fix round 3) only partly: see "Mesopic degree of adaptation" and the fix
  round 3 numbers. The full
  Moon at a VR fov stays display white (B/R 1.00 in A and B): adapted to the moonlit night (L_a 0.04 cd/m^2, gain 1.2e4)
  the disc is far above white - the Moon's own disability glare (10 x 0.3 lux / 0.27^2 ~ 40 cd/m^2 looking at it) is
  added as a CIE 146 veil (T13); local adaptation (T10) now brings a disc that is resolved (>= ~2 blocks wide) into range: fov 12 at
  gain 6645, disc sRGB (255, 255, 255) -> (233, 211, 189) with the maria; pale cream (B/R 0.81) is the white balance's
  (the adopted white of the moonlit night with the mesopic D), not the local step's. At fov 100 the 3-4 px disc keeps
  its look. The surround
  model has no moonlit sky (black at night), so at night the fov-2 frame still stands alone (out-of-frame share 0).
- Earth in view for a Moon eye at partial phases underestimates up to ~2x; the eclipse-time Earth renders several
  times its day brightness (AgX rolls it toward white) - kept physical.
- Earth in view and the earthshine ignore the clouds the Earth is drawn with (Task 10: the scaled-space Earth shows
  them): the eye adapts to the cloud-free model ground (the Sun's plate dominates wherever the eye is sunlit; on the
  lunar night side the cloudy Earth is brighter than adapted to); the earthshine's Bond albedo 0.306 holds the clouds
  on average. Follow-up: the 13 directions through the cloud column.
- Eclipsed-Moon corona not modelled. Local adaptation (T10) is gaze-free (every region adapts locally at once; no foveal
  weighting or eye tracking).
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
- Local adaptation (T10): a small bright object stays inside the field rule's 2 % highlight cut (the global gain is
  unchanged) and is brought into range by the local step instead. Its limits: (1) a structure narrower than ~2 blocks
  (a thin Earth crescent at desktop resolution, 1-2 px; a 3-4 px full Moon at fov 100) finds no like block and keeps c
  = 1 or pops with the grid phase (the desktop and the headset therefore look different for the same object by design) (stays white; the headset has ~4.8x the desktop's pixels per degree); (2) the Earth's / Moon's disability
  glare (CIE 146: full Earth from the lunar night 6.9 lux at the eye, 18 / 6.4 / 1.3 / 0.46 cd/m^2 at 2 / 3 / 6 / 10 deg;
  full Moon 0.25 lux, ~0.64 / 0.074 / 0.017 cd/m^2 at 2 / 5 / 10 deg against a moonlit sky of ~0.02 cd/m^2, Krisciunas &
  Schaefer 1991) raises the adaptation at full strength but is shown at its source's local factor ("Glare spread"); the
  eye's far veil (where it is below the field, > ~10 deg) is therefore not visible - whether it should blend to the
  global gain there is open; (3) display-referred
  emitters over a locally adapted region take its factor; (4) Godot's glow on other sources (Sun disc, plume, lamp
  lenses) is unchanged and still not tied to CIE 146; (5) the Earth's locally adapted disc is light blue with the
  ocean at the gamut edge (blue channel 254 in ~55 % of its pixels at fov 100), as when the Earth fills the view.
- The VR panel was not in the stereo mock's lunar-night view (`--keep_vr_panel`), and the night plume seen from
  500 m (fov 25 deg) stays inside the 2 % highlight cut; the run-away the meter now avoids was therefore not
  reproduced in a render, only excluded by construction (display-referred shaders in the transparent pass, test).
