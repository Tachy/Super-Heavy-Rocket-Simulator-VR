# Eye adaptation, night vision and Moon light (M8)

History and the reasons behind superseded decisions: `docs/history/atmosphere-rendering.md`.

Plan `docs/superpowers/archive/plans/2026-10-24-m8-atmosphere.md`, spec
`docs/superpowers/archive/specs/2026-10-24-m8-atmosphere-design.md`.
Part of the atmosphere docs; index `docs/subsystems/atmosphere-rendering.md`. Tables and
shaders:
`atmosphere-scattering.md`.

## Model

- **Eye adaptation** [call 2026-10-25, `eye_adaptation.gd`, fidelity flag `Atmosphere.eye_adaptation_enabled`, on in
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
    The lunar ground sphere is the mean sphere, or 2 m below an eye standing below it. Values: lunar day 1.000 (Moon
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
    colour, before glow and AgX: rgb' = Y + S (rgb - Y) + k g_r / g_c x (w_s . rgb) x tint (Y the BT.709 luminance),
    per pixel on every view layer; nothing dispatched while k g_r / g_c = 0 and S = 1.
  - Display-referred emitters write the inverse of that linear step (Sherman-Morrison): q = (c - (1 - S) Y) / S, then
    q - u (w_s . q), u = s C^-1 tint / (1 + s w_s . C^-1 tint) (`vision.gdshaderinc`, globals `vision_rod_weights`,
    `vision_inverse_tint`, `vision_chroma`): plume, the stars' own brightness, the orbit ribbon, flight trail, the
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
    post step, which also runs for chroma alone whenever L_a < 1e4 cd/m^2.
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
- View-independent adaptation (VR-stable); Earth-in-view stays physical (no display-side highlight treatment).
- City lights scene-referred with scale [E] (user ruling 2026-10-25).

## Open points and known limits

- Earth in view for a Moon eye at partial phases underestimates up to ~2x; the eclipse-time Earth renders several
  times its day brightness (AgX rolls it toward white) - kept physical.
- Earth in view and the earthshine ignore the clouds the Earth is drawn with (Task 10: the scaled-space Earth shows
  them): the eye adapts to the cloud-free model ground (the Sun's plate dominates wherever the eye is sunlit; on the
  lunar night side the cloudy Earth is brighter than adapted to); the earthshine's Bond albedo 0.306 holds the clouds
  on average. Follow-up: the 13 directions through the cloud column.
- Eclipsed-Moon corona not modelled; no per-pixel / foveal adaptation.
