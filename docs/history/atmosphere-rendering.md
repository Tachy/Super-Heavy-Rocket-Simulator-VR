# Atmosphere rendering (M8) - history

Verbatim record of this subsystem's notes as they stood in `docs/subsystems.md` on 2026-10-25, before the split. Current state: `docs/subsystems/atmosphere-rendering.md`. Kept for the reasons behind decisions, superseded numbers and earlier approaches; read only when that history is needed.

Plan `docs/superpowers/archive/plans/2026-10-24-m8-atmosphere.md`, spec `docs/superpowers/archive/specs/2026-10-24-m8-atmosphere-design.md`.
One physical model of the Earth's atmosphere for every viewpoint - pad, ascent, orbit, the Moon - so the sky, the limb
and the haze change continuously with the camera's position, direction and the Sun; colours come out of the spectral
physics, nothing is tuned. Flight physics is untouched (the vehicles still fly through `Us76Atmosphere`).

- **Model** (core, `environment`): a float64 port of Bruneton's "Precomputed Atmospheric Scattering: a New
  Implementation" (2017, BSD-3; ported files carry his notice, CREDITS.md ships it in full) in precomputed-luminance
  mode: 15 wavelengths 360-830 nm in five passes of three, folded with the CIE 1931 2-degree functions into XYZ and
  linear sRGB (D65), no white balance (the TOA Sun is slightly warm-white). Inputs `config/atmosphere/earth_optics.toml` + spectra in
  `atmosphere_spectra.hpp`: Rayleigh from the refractive index of air (Peck & Reeder 1972) with the King factor
  (Bucholtz 1995), the US76 number density as profile; ozone cross sections (Serdyuchenko 2014 via Bruneton) on the
  AFGL US standard profile, 300 DU [S]; aerosol AOD 0.1 at 550 nm, Angstrom 1.3, single-scattering albedo 0.9, scale
  height 1.2 km, Cornette-Shanks g 0.8 [E]; solar spectrum ASTM G-173 ETR (via Bruneton; scaled by 1 / d_AU^2 at run
  time); ground albedo 0.1 [E]; top 100 km [call], model ground `ground_altitude_m` -150 m [E] (below the geoid minimum
  -106 m and every pad; cameras and points below are clamped onto it). Code: `atmosphere_optics.*` (coefficients),
  `atmosphere_tables.*` (parameterisation, input hash), `atmosphere_precompute.*` (the precomputation, parallel over
  table rows), `atmosphere_scattering.*` (CPU evaluation: sun transmittance, sky luminance, sky luminance to a point,
  sun + sky illuminance), `atmosphere_table_file.*` (cache).
- **WGS84.** Bruneton's model is spherical; it is evaluated in a space where the ellipsoid becomes the sphere of
  radius a (polar axis x a/b = 1.00336): the ground matches the ellipsoid everywhere (test: < 1 mm at 0/30/60/90 deg),
  optical paths are off by at most 0.34 %. Rays and the Sun map by that stretch, normals by its inverse transpose; the
  direct-sun cosine is taken in true ITRF geometry. CPU and shaders use the same mapping.
- **Multiple scattering.** Five orders computed (Bruneton: 4) plus a **geometric tail** of the higher orders
  (Hillaire 2020, Eq. 9): the series is geometric from order 3-4 on (per-order ratio q 0.39-0.44); tail = last order x
  q / (1 - q), q = last / previous per texel and wavelength, capped at 0.75. Plain truncation after 4 orders was 7.2 %
  short of 14 orders 2 deg above the horizon (2.9 % at the zenith). Fidelity flag `multiple_scattering_tail` in
  `earth_optics.toml` (true; false truncates like Bruneton; part of the input hash; test: on in the shipped config and
  it changes the tables). 5 vs 7 orders with the tail: within 0.20 % (sky illuminance 0.03 %).
- **Tables and cache.** Transmittance 1024 x 1024 (Bruneton's 256 x 64 erred 82 % on grazing rays), scattering
  (Rayleigh + all multiple orders) and single Mie 8 nu x 32 mu_s x 128 mu x 64 r (64 r rows: off-texel error 2.8 % ->
  0.71 %), irradiance 64 x 16; mu_s_min = cos 120 deg (twilight to -18 deg for the star threshold). Precompute 61-62 s
  on 24 threads (once per input hash), file `data/atmosphere/earth_atmosphere.bin` (83.9 MB: header with magic,
  version, input hash, sizes and parameters, four float32 arrays, FNV-1a 64 checksum verified before parsing; written
  to a temp name and renamed; a damaged, stale or missing file is recomputed, a failed write still returns the tables).
  The input hash covers the config, the table sizes, the stored spectral / optics data (machine-independent: no libm
  outputs) and `atmosphere_tables_code_version`. CLI `build/core/atmosphere_tables.exe [--config] [--out]`;
  `tools/package.ps1` runs it, so installed games ship the cache. The game loads the cache on a worker
  (`StarsimCore.load_atmosphere`; on a first run it computes and shows "Atmosphäre wird berechnet (einmalig, etwa eine
  Minute) ..." in the time panel). GPU formats: transmittance RGBAF (its grazing values ~1e-5 are half-float
  subnormals; RGBAH put a 20 km Cape pixel 4 % off in blue), scattering / Mie / irradiance RGBAH (half the VRAM; GPU
  check worst 0.52 % vs 0.47 % with RGBAF); ~50 MB VRAM.
- **Shader include** `game/shaders/atmosphere.gdshaderinc` (port of the CPU code, globals in project.godot
  `[shader_globals]` `atmosphere_*`): `atm_sky` (sky radiance + transmittance along a view ray), `atm_to_point*`
  (aerial perspective between the camera and a point; camera-relative offsets for near points), `atm_sun_at` /
  `atm_sky_light` / `atm_sun_and_sky` (light on a surface), `atm_sky_to_far_object` (Moon, Sun), `atm_fog` (vehicles).
  Numerics for the floating origin: the camera's Earth-local position arrives as float32 hi + lo
  (`atmosphere_camera_hi/lo`), its altitude above the model ground differenced in float64 on the CPU
  (`atmosphere_camera_altitude_m`); rho = sqrt(h (2R + h)), discriminants as (r mu)^2 - rho^2 and (T - r)(T + r), a
  point's height along a ray as h + d(d + 2 r mu) / (r_d + r) - never r^2 - R^2 of large float32 values. Cameras above
  the top enter through the ray's closest approach (the limb is right from the Moon).
- **Where it is used.** Sky: `sky.gdshader` (Environment BG_SKY, every pixel from the camera's true position; the
  radiance cubemap also draws the model ground lit by the model, so Godot's ambient and reflections carry sky light,
  ground bounce and earthshine; PROCESS_MODE_INCREMENTAL, radiance 128). Earth: `earth.gdshader` (also scaled space),
  `terrain_earth.gdshader`, `water.gdshaderinc` (the sea's reflected sky is the model's sky along the reflected ray
  from the water point; `sky_radiance` is gone; a ray reflected below the horizontal takes its upward mirror [E]): the
  Sun transmitted to the point x cos + the sky irradiance on the normal, then transmittance + in-scattering to the
  camera; city lights x T. Moon, Sun disc: x T + the in-scattered sky (the daytime Moon is pale; the Sun reddens at the
  horizon). The Sun disc's colour is the model's TOA solar spectrum in the luminance mode
  (`atmosphere_sun_illuminance_lux`, the colour of the Sun light and the ground lighting), normalised to unit luminance
  so the disc's display brightness (`intensity`) is unchanged (final review M6; was a hand-picked warm white). Stars
  (`star_visibility.gdshaderinc`): x T and hidden below the eye's detection threshold over the background,
  I_th = c1 (1 + sqrt(c2 B))^2 (Schaefer 1990, PASP 102, 212: night / day constants, smooth step +-0.5 mag [E]);
  limiting magnitude 7.9 at B = 0, about -4 at the noon sky. B = max(the sky behind the star, the luminance the eye is
  adapted to) (final review I1; Schaefer's B is the adapting luminance, and in a non-uniform field the adaptation
  dominates over a dark patch: Ferwerda et al. 1996). The adaptation luminance is `EyeAdaptation.adaptation_cd_m2`,
  paced like the gain (light 0.5 s, dark 110 s, log space) but unclamped (floor 1e-6 cd/m^2 [call], far below a dark
  night sky), global `atmosphere_adaptation_cd_m2` (0 with the adaptation off). Consequence: sunlit space views are
  starless - the day-side orbit (the eye adapted to the Sun and the day-side Earth, L_a 4.2e4 cd/m^2 at 400 km:
  limiting magnitude about -6; GPU check: magnitude 0 and 4 hidden beside the limb, visible with the fixed exposure),
  cislunar space with the Sun on the eye - as astronauts report and cameras show; stars appear on the night
  side or in the Earth's shadow after dark adaptation (tau 110 s, minutes from day to the faint stars), in twilight and
  in totality (L_a there is below the sky behind the star). The Moon and the Sun disc need no threshold (physical
  luminances x the gain, spec Goal 6); city lights are scene-referred luminances since the night-vision fix round
  (see "Night vision"); planets are not drawn. At night the moonlit sky behind a star counts too.
  Vehicles, tower, pad, drone ship: `lit_aerial.gdshader` (`LitAerial.material`): Godot's
  PBR lighting plus aerial perspective through FOG (Godot blends FOG with one scalar, so the per-channel T is split:
  t = max(T), FOG.a = 1 - t, FOG.rgb = S / (1 - t), and T / t on albedo and emission - diffuse, metal and emission end at
  lit x T + S exactly; only the dielectric specular takes the scalar); the tower lattice through `atm_fog`. Plumes
  (`plume.gdshader`, additive): x T only [call]. The flight trail, orbit lines and labels are UI markers, not attenuated
  (and past night vision's rod step unchanged: `vision.gdshaderinc`; the map labels are `MapLabel` TextMeshes, the Moon's
  orbit line `display_colour.gdshader` - Label3D / StandardMaterial3D cannot take that inverse).
- **Sun light.** One DirectionalLight: colour and energy per channel = TOA solar RGB x the core's
  `atmosphere_sun_transmittance` x luminance scale / pi x the Moon's visible fraction, at **one point: the followed
  vehicle** (vehicle focus), else the camera (`Atmosphere.sun_light_at`, `world.gd::_update_sun_light`). Near the Moon
  the visible fraction comes from `Atmosphere.moon_visible_fraction`, shared with the eye adaptation: within 2 m of the
  lunar mean sphere or below it (terrain under the mean radius, a lander standing there) the Moon's occultation is
  taken on the point's own ground sphere - the core's mean-sphere fraction hid the Sun from it at lunar day (fix
  2026-10-25; GPU check: 1551 m below the mean radius, Sun 60 deg up: mean-sphere fraction 0, light = the open sky's). Known limit:
  other bodies in view (the booster after separation far below the ship, the tower seen from orbit) get the light of
  that point - a body at sunset next to one in daylight is lit like the followed one. Per-object lights would cost a
  light per body; not done.
- **Exposure.** One exposure for everything: scene value = luminance x `DAY_EXPOSURE` (1.5) x pi / E_v,TOA / d_AU^2 - a
  white Lambertian under the TOA Sun renders 1.5 in every material. **Eye adaptation** [call 2026-10-25,
  `eye_adaptation.gd`, fidelity flag `Atmosphere.eye_adaptation_enabled`, on in the game, off in the GPU check's
  comparisons; night vision by the user decision of 2026-10-25, see "Night vision" below]: the cone gain on every
  physical luminance (tables, Sun light, Moon, Sun disc, city lights; plumes and the stars' own brightness are
  display-referred). Adaptation luminance L_a = the largest of: down = a white plate at the eye under the transmitted
  Sun at normal incidence and the sky, (E_sun,n + E_sky) / pi x the visible fraction (+ the Moon's plate at night);
  up = the Earth in view (`Atmosphere.adaptation_luminance`, see below); the twilight glow (night vision). Cone gain =
  L_ref / (10^1.255 t_p(L_a)) in [1, 1e5], L_ref = the plate under the TOA Sun, t_p Ferwerda's cone t.v.i.: above ~80
  cd/m^2 exactly the Weber gain L_ref / L_a of before (the day look unchanged), below it the cone threshold flattens.
  The former cap 25 (a display call) is gone; 1e5 is a numeric bound only (the t.v.i. floor bounds the gain at
  ~1.24e4). Pacing [E]: light adaptation tau 0.5 s, dark adaptation 110 s (Pattanaik et al. 2000), exponential in log
  luminance (the photopic and the scotopic adaptation luminance); time step = max(frame time, simulated step)
  (paused: real time, time warp: a time-lapse, a clock jump or a view change: adapted at once); the photopic target is
  capped at L_ref before pacing (gain 1 above it anyway; an eye paced from above L_ref would otherwise wait at the dark
  constant before the gain moved; review M3). Values: pad noon gain 1.10, 400 km by day 1.000 (both as before; GPU
  check: = L_ref / L_a within 1e-9), Sun -3 deg at the pad 464 (adapted to half the glow, 90.8 cd/m^2).
  The up part: 13 directions below the eye in model space (nadir, a ring at 0.6 x the Earth disc's angular radius, a
  ring grazing the limb 10 km up), each the plate equivalent of what it shows (ground illuminance / pi x T + in-scatter,
  or the limb's sky, x the Sun's visible fraction there), times the view factor (R / r)^2; view-independent (VR-stable).
  While the Moon's penumbra touches the atmosphere the sky samples are segmented like the umbra fluence (8 segments,
  each x the visible fraction at its midpoint; ~0.7 ms per evaluation, eclipse only; final fix wave): one factor at the
  ray's closest approach - behind a ground eye looking up - had put the totality eye at 56 cd/m^2 (the uneclipsed
  horizon sky beyond the umbra); the Earth in view of a Dallas eye 2 m above the terrain is 1.90 cd/m^2 (the former
  "1.57 / gain 5150" was rendered from 0 m on the ellipsoid, ~107 m inside the terrain: night-vision review I5,
  render_check's ground view now stands on the core's surface). In totality L_a is the glow, below. Outside an
  eclipse no shadow test runs at all (`penumbra_touches` first: no CSPICE calls in the adaptation; final review M1).
  At 400 km on the 2024-04-08 umbra axis: down 0, up 2390 cd/m^2 (Task 10: 2428 with the closest-approach factor; the
  same Earth uneclipsed: 20871), gain 17.7. Not a quadrature or weighting artefact: a dense cap integral (48 rings x 24
  azimuths; Task 10) gives gain 15.1
  cosine-weighted (the plate) and 17.0 weighted by solid angle; the ring means rise from 494 cd/m^2 at the nadir to
  6344 at the limb - the visible Earth lies in deep penumbra (visible fraction 0.03 / 0.10 / 0.33 / 0.65 at 200 / 400
  / 1000 / 2000 km from the umbra's centre). The eye has no Sun in it there and adapts to that dim Earth, so the
  outer, less eclipsed Earth renders several times its day brightness (AgX rolls it toward white). Cameras on the ISS
  show it at a day exposure; an adapted eye does not. Kept physical [call]; a display-side highlight treatment would
  be a separate decision.
  **Away from the Earth** (Moon-eye fix 2026-10-25; only for an eye above the top of the atmosphere, nothing changes
  inside it). Down: the transmittance is 1 there (or the grazing ray's; no clamp issue beyond the tables, the sky
  irradiance is the top boundary's ~0), and the visible fraction also takes the Earth's disc in front of the Sun (the
  transmittance alone is the Sun's centre ray: a sharp shadow edge far out; where both act, within the Sun's radius
  of the limb, the penumbra is counted twice [call]). Up: the Earth in view x the part of its disc the Moon leaves (a
  lunar eye with the Earth below its horizon). New third term, the **Moon in view**
  (`Atmosphere._moon_in_view_luminance`, needs the Moon's geometric centre: `update(..., moon_gcrf_m)`): the regolith
  luminance as the Moon shaders draw it - mean texture albedo 0.30 [D: `moon_color.png` area-weighted, sRGB-decoded,
  BT.709] x the regolith reflectance (see "Moon disc and regolith" under Night vision: R x LS x f(g) for the Sun, x
  the solar disc the Earth leaves at the ground point - lunar eclipse, the umbra glow left out - and the same for the
  earthshine) - averaged cosine-weighted over the lunar cap (4 rings uniform in sin^2 x 8 azimuths) x its view factor
  (R / r)^2. Earthshine (`Atmosphere.earthshine_relative`, the same value the Moon shaders
  draw, see "Earthshine" below). L_a = the largest of
  down, Earth, Moon. The lunar ground sphere is the mean sphere, or 2 m below an eye standing below it (terrain under
  the mean radius); there the Sun's visibility is recomputed on that sphere (the core's mean-sphere fraction would
  hide the Sun from it; e.g. 0 N 45 E lies 1551 m below the mean radius; `Atmosphere.moon_visible_fraction`, also
  used by the vehicles' Sun light). Values: lunar day 1.000 (the Moon term
  3264 cd/m^2 with the K&S phase law, formerly ~7800; the Sun plate 42 400); deep space halfway to the Moon 1.000;
  lunar night under a nearly full Earth
  (phase 10 deg, check): Earth in view 6.17 cd/m^2 (0.80 x the Lambert disc 7.74: the 10 km limb ring carries 25 %
  of the weight with only the limb's sky), Moon in view 0.20, earthshine 7.35 lux, cone gain 2845 (Weber 6872; the
  former cap 25); the earthshine-lit ground shows dark grey, the full Earth is blown to white (expected) [call: the
  view-independent plate model has no foveal term - an eye fixating the Earth would adapt partly to it]. Away from the Earth at partial phases the
  13-direction cap underestimates the Earth by up to ~2x (half Earth: the nadir sample on the terminator, the outer
  annulus represented by the limb's sky); it only matters where the cap 25 holds anyway.
  **Why render_check's `--view_from=moon` showed gain ~7.5:** not a lunar eye - `--focus=earth --view_from=moon` is the
  orbit camera 22 000 km from the Earth's centre on the Moon side. `_view_from` ran `update_world` to read the Moon's
  direction before aiming, which used up set_focus's snap at the default pose (night side: down 0, Earth in view
  ~0.02 cd/m^2, gain 25); after aiming, light adaptation (tau 0.5 s) over the ~0.24 s of settle frames left
  25 -> 11.1 -> 7.9 -> 7.5. Fix: render_check snaps after the view is placed (frame 0, before the world's own update).
  In the game every view change (`set_focus`, `set_ground_site`, Tab, mission start) snaps before the next
  `update_world`, which places the camera before the atmosphere update.
- **Earthshine** (fix 2026-10-25; replaces the Moon shaders' constant `earthshine = 0.004`, ~360 lux equivalent on an
  albedo-1 surface and phase-blind). `Atmosphere.earthshine_relative`: the Earth a Lambertian sphere of the Bond
  albedo 0.306 [S: NASA Earth Fact Sheet] (geometric albedo 2/3 A [S: Lambert-sphere phase integral 3/2]) at its phase
  angle a (Sun - Earth - Moon), illuminance on a plate at the Moon's centre facing the Earth / the TOA Sun at 1 AU =
  (2/3) A (R / d)^2 (sin a + (pi - a) cos a) / pi / d_AU^2 - pi x A x the Lambert disc's plate luminance, the quantity
  the eye adaptation's Earth-in-view cap approximates (closed form here: the cap reads 0.8 x at full, ~0.4 x at half
  Earth from the Moon). Full Earth at 384 400 km: 5.6e-5 = 7.46 lux (the tables' TOA illuminance); measured estimates of
  the full earthshine run ~7-15 lux (the real Earth is not Lambertian). Clouds, sun glint, city lights, the near
  side's 0.5 % shorter distance: not modelled. Once per frame in `Atmosphere.update` (needs no tables), the global
  shader uniform `moon_earthshine` = DAY_EXPOSURE x that (a white Lambertian facing the Earth, scene units without the
  exposure gain - the Moon shaders multiply the whole Moon by `atmosphere_exposure_gain`); `moon.gdshader` and
  `terrain_moon.gdshader` draw albedo x R x moon_earthshine x LS(mu_e, mu) x f(g_e) - the sunlight's regolith BRDF
  (night-vision review I3: the former Lambert earthshine was 1.9 x too bright against the sunlit regolith); the
  reddened umbra glow is unchanged. The eye adaptation's Moon-in-view term and `last_earthshine_lux` use the same
  function. GPU check (section 12): full Earth 7.46 lux (band 5-15 lux [call]: catches a unit or phase error, not the
  albedo choice), the Moon shader's dark side at the sub-Earth point seen from the Earth's side = albedo x
  moon_earthshine x R x f(0) (0.513 x 1.35 = 0.6925: the earthshine is backscattered at opposition) within 3 % at full
  Earth and at phase 60 deg (ratios 0.99).
  **Display (resolved by night vision, 2026-10-25):** with the former cap 25 the earthshine-lit regolith (~0.4-0.6
  cd/m^2) rendered black. Now the lunar night ground under a nearly full Earth (0 N 45 E, 2026-11-07T14:00, cone gain
  3012) is dark grey, and the Da Vinci glow of a 2-day crescent from Starbase at Sun -8 deg (2026-11-12T00:15, adapted
  to half the glow, 1.27-1.31 cd/m^2, gain 5610-5670) lies 16 % above the twilight sky beside the Moon (GPU check,
  needs > the cones' Weber fraction 5.6 %; 19.9 % before the fix round's BRDF and half-glow changes).
- **Night vision** (user decision 2026-10-25, "like the eye"; `eye_adaptation.gd`, `vision_effect.gd`,
  `vision.gdshaderinc`). Night views look as a human perceives them: dim, desaturated and bluish at low luminance, a
  moonless night dark. Model [call: chosen over Pattanaik et al. 2000 and Kirk & O'Brien 2011]: Ferwerda, Pattanaik,
  Shirley & Greenberg 1996, "A model of visual adaptation for realistic image synthesis" (SIGGRAPH 96) - Ward's
  threshold matching (a luminance at the world observer's threshold maps to the display observer's) with separate
  t.v.i. functions for cones (Eq. 4: log t_p = -0.72 below log L_a -2.6, log L_a - 1.255 above 1.9, (0.249 log L_a +
  0.65)^2.7 - 0.72 between) and rods (Eq. 5: -2.86 below -3.94, log L_as - 0.395 above -1.44, (0.405 log L_as +
  1.6)^2.18 - 2.86 between), combined (Eq. 6) display = m_c L + k m_r L_s. Here m_c = the cone gain (above) and m_r =
  L_ref / (10^1.255 t_s(L_as)) (the display observer photopic, as Ferwerda's m_r = t_p(L_da) / t_s(L_wa); the display
  adaptation is the day look's) [call]. Weber's law holds for the cones above ~80 cd/m^2 (unchanged day) and fails at
  scotopic levels (the thresholds flatten): a scene without light stays black however dark-adapted the eye.
  - k = 1 - s, s the rods' saturation of Jensen, Premoze, Shirley, Thompson, Ferwerda & Stark 2000 ("Night rendering",
    UUCS-00-016, sec. 3.1): 0 below 0.01 cd/m^2, 1 above 4 cd/m^2, 3 x^2 - 2 x^3 with x = (log L + 2) / 2.6 between -
    evaluated on the photopic adaptation luminance [call: one k per view, VR-stable].
  - Scotopic luminance: L_s = w_s . rgb, w_s = (0, 1.4730, 0.9238) [D: `tools/validation/scotopic_rgb_weights.py`:
    the CIE 1951 V'(lambda) (K'm 1700 lm/W) projected onto the linear sRGB colour matching functions, least squares in
    S/P over black bodies 2000-25 000 K and their lambda^-4 sky versions, every weight >= 0; worst 4.2 % (the 25 000 K
    sky), illuminant A +1.4 %, 6504 K -1.7 %; S/P of RGB white 2.40]. Non-negative since the brightness term
    (2026-10-25) [call]: the post step is now linear (no clamp of w_s . rgb), so a scene pixel's scotopic sum must not go
    negative; the unconstrained fit (-0.0525, 1.6016, 0.8794; worst 2.2 %) would let a pure red pixel subtract rod
    light. The scotopic adaptation luminance L_as is the chosen plate's colour through w_s.
  - The rods' image is achromatic, drawn with a blue tint at chromaticity (0.25, 0.25) (Jensen et al. 2000: their
    rendering choice beside the mean (0.29, 0.28) of ten night paintings; the night convention also of Thompson,
    Shirley & Ferwerda 2002, JGT 7(1)) [call: the measured mean is purple and gave a lavender twilight sky].
  - Where: the cone gain in the shaders (as before); the rods by one CompositorEffect (`VisionEffect`, attached to the
    WorldEnvironment's Compositor in world.gd and the GPU check) after the transparent pass with the resolved MSAA
    colour, before glow and AgX: rgb' = Y + S (rgb - Y) + k g_r / g_c x (w_s . rgb) x tint (S the chroma factor of the
    brightness term below, Y the BT.709 luminance), per pixel, on every view layer (multiview: identical for both
    eyes); nothing dispatched while k g_r / g_c = 0 and S = 1 (daylight, the adaptation off). The gain stays in the
    shaders as a pre-exposure, so the half-float buffer holds values near display level.
  - Display-referred emitters write the inverse of that linear step (Sherman-Morrison): q = (c - (1 - S) Y) / S, then
    q - u (w_s . q), u = s C^-1 tint / (1 + s w_s . C^-1 tint) (`vision.gdshaderinc`, globals `vision_rod_weights`,
    `vision_inverse_tint`, `vision_chroma`): plume, the stars' own brightness, the orbit ribbon, the flight trail, the
    Moon's orbit line, the map labels and the VR panel look as before (they do not take the brightness factor either:
    it sits in the scene's pre-exposure). The step is linear, so the inverse holds under alpha and additive blending
    too (with the former clamp max(w_s . rgb, 0) a desaturation inverse would have oversaturated warm emitters into a
    negative scotopic sum and dropped the background's rods behind them). GPU check (review I7): an opaque
    `display_colour` quad at the moonless night (rod scale 138, S 0.52) = its day pixel within 0.5 % of its peak, a
    half-transparent one over a scene-referred background = alpha x colour + (1 - alpha) x the background through the
    step within 0.5 % of the peak - the limit is rgba16f: the inverse writes channels whose scotopic sum nearly
    cancels, and the step multiplies its half-float rounding by s x tint (tolerance 2 % of the peak).
    Plumes stay display-referred [call]. The map labels (`MapLabel`) draw Label3D's black outline as eight black
    copies 2.5 font pixels out, under the text (review M2). The Sun disc clamps its scene value at 6e4 (half-float
    range when the Sun appears in a dark-adapted view; the compute step also clamps and drops NaN).
  - City lights (user ruling 2026-10-25 in the fix round, review I6): scene-referred, a luminance through the eye model
    like every other: L = `Atmosphere.CITY_LIGHTS_PEAK_CD_M2` x the night texture's sRGB-decoded texel squared (the
    former shaping), x T and the aerial perspective like the ground (global `earth_city_lights_cd_m2`; `earth.gdshader`,
    `terrain_earth.gdshader`; before the tables load the former display value). The Black Marble 2016 texture is an
    8-bit visualisation without radiometric units, so the scale is [E]: a saturated white texel - a bright urban core
    averaged over its 4.9 km texel - is 0.3 cd/m^2 upward luminance, anchored to lit roads (0.75-1.5 cd/m^2, EN 13201-2
    M3-M2, covering a fraction of a city) and to VIIRS DNB radiances of bright cores (~100-300 nW/(cm^2 sr), ~0.1-0.4
    cd/m^2 with ~260 nW/(cm^2 sr) per lm/m^2 emitted, arXiv 2510.02977's site calibration). The orange 4.9 km veil
    over the moonlit Cape is gone (the lights now sit ~2.5 x above the 0.12 cd/m^2 adaptation of a full-Moon night
    instead of at a fixed display level ~50 x above the moonlit ground); from orbit the cities show as pale, rod-tinted
    patches (one k per view: no foveal colour for bright lights). The texture stays at 8192 x 4096 (4.9 km/px);
    follow-up: Black Marble VNP46A4 at 15 arcsec (~500 m) near the sites, masked by the imagery's urban footprint.
  - Glow (`Atmosphere._twilight_glow_luminance`, `_limb_glow_luminance`) [call: view-independent]: Ferwerda takes half
    the highest visible luminance as the world adaptation, and the glow enters L_a at `GLOW_ADAPTATION_SHARE` 0.5 of
    the brightest sky (review M5; formerly the full brightest sky). Twilight: with the Sun below the eye's horizon,
    the brightest sky along the Sun's azimuth (elevations 1-40 deg, at least 8 deg from the Sun, outside its aureole;
    azimuth-only sampling is an accepted limit, review M6). Solar eclipse at the eye (its own visible fraction < 1,
    review I2 - formerly any penumbra on the Earth switched the glow off for every observer for hours): the same rule
    over 8 azimuths x 1 / 3 / 6 / 12 deg of the eclipsed sky (`_eclipsed_in_scatter`), the ring around the umbra.
    Orbit: above the atmosphere with the Sun behind the Earth and the eye not near the Moon (its disc smaller than the
    Earth's; review I1 - formerly any passed Moon disabled it, so it never ran in the game), the brightest limb toward
    the Sun (tangent heights 2-40 km). Without the glow the eye adapted to the dim overhead plate and the glow over the
    western horizon rendered white (Sun -3 deg: gain 914); with half the glow the -3 deg zenith rendered 1.9 x the noon
    zenith on screen (Weber range: the dusk zenith maps against half the bright western sky) - the suprathreshold
    dimness of dusk is not in a threshold model (review (d)); the brightness term below adds it.
  - Brightness and colourfulness (user decision 2026-10-25, `EyeAdaptation.brightness_factor` / `chroma_factor`):
    threshold matching is ratio-preserving in the Weber range (g_c = L_ref / L_a), so every adapted scene there looked as
    bright and as colourful as noon. A real eye sees a dim scene dimmer (Stevens effect) and less colourful (Hunt
    effect) even when adapted. Model [call: the simplest view-independent one, chosen over Stevens & Stevens 1963 /
    Tumblin-Rushmeier, Pattanaik et al. 2000 and Irawan et al. 2005]: CIECAM02's luminance-level factor F_L [S: CIE
    159:2004; Moroney et al. 2002, "The CIECAM02 color appearance model", IS&T/SID CIC 10], F_L = 0.2 k^4 (5 L_A) +
    0.1 (1 - k^4)^2 (5 L_A)^(1/3), k = 1 / (5 L_A + 1); at a constant relative stimulus (the eye adapted: J, C fixed)
    brightness Q ~ (A_w + 4) F_L^0.25 and colourfulness M = C F_L^0.25 (A_w the adapted white's achromatic response,
    itself growing with F_L^0.42). Brightness factor B = Q_w(L_a) / Q_w(L_day) on the displayed luminance [call: the
    brightness ratio applied as a luminance ratio - matching it through the display observer's ~1/3-power brightness
    would ask for B^3, 6e-5 for the full-Moon cone image], chroma factor S = (F_L(L_a) / F_L(L_day))^0.25 on the cones'
    chroma at constant luminance; L_A = L_a x Y_b / 100, Y_b = 20 (L_a is the scene's white: the sunlit plate, half the
    brightest sky) [call]; both 1 at and above L_day = 1e4 cd/m^2 [call: a white plate under ~3.1e4 lux, the Sun ~15
    deg up; noon at the pad, L_a 3.8e4, is identical]. Held so nothing is counted twice: B below the Weber range (79
    cd/m^2: Ferwerda's flattening cone threshold dims the display from there; the product stays monotonic in L_a), S
    below the mesopic upper limit (4 cd/m^2: the rods' achromatic share takes over the desaturation, and the inverse's
    1 / S amplification of display-referred emitters stays bounded in rgba16f). Values (L_a -> B, S): >= 1e4 -> 1, 1;
    3264 (a regolith-only view) -> 0.79, 0.91; Sun -3 deg (90 cd/m^2) -> 0.378, 0.676; <= 79 -> B 0.368; <= 4 -> S 0.521. Where: B
    multiplies the cone gain's pre-exposure (`luminance_scale`, `scene_per_cd_m2`, `atmosphere_exposure_gain`; the
    `exposure_gain` field stays the cone gain), so the whole displayed image dims - the rods, relative to that pixel,
    follow - while the star threshold, which reads the sky's true luminance back through `scene_per_cd_m2`, is
    unchanged; S acts in the post step. GPU check (section 10): the -3 deg zenith 0.726 x the noon zenith on screen
    (was 1.92), its displayed / scene chroma 0.675 against noon's 1.000; the order noon 0.148 > -3 deg 0.108 > -8 deg
    0.0031 > -12 deg 0.0015 (displayed zenith luminance). The -3 deg zenith's own colour is ~1.6 x as saturated as the
    noon zenith's (the deep twilight blue): after S it is still 8 % more colourful on screen (0.646 against 0.596), so
    the check compares the displayed chroma relative to the scene's.
  - Moon disc and regolith (`moon_regolith.gdshaderinc`, mirrored by `Atmosphere.regolith_reflectance`; reviews I3,
    I4, M7): one BRDF for every light on the regolith, albedo x R x E / pi x LS(mu0, mu) x f(g), LS = 2 mu0 / (mu0 +
    mu), g the local phase angle, f(g) = O(g) 10^(-0.4 (0.026 g + 4e-9 g^4)) / Phi_LS(g) with the opposition surge
    O = 1.35 - 0.05 g below 7 deg [S: K&S 1991, as Schaefer's skycalc applies it] and Phi_LS(g) = 1 - sin(g/2)
    tan(g/2) ln cot(g/4) the Lommel-Seeliger sphere's disc-integrated phase function (checked numerically by
    `tools/validation/moon_disc_calibration.py`); g held at 150 deg beyond [call]. A normal facing away from the eye
    (mu < 1e-3: the interpolated vertex normals at the faceted silhouette, MSAA samples beyond a triangle) is taken onto
    the limb (its eye component raised to 1e-3) before mu0 [call, 2026-10-25]: in the 2024-04-08 totality the Sun just
    behind the limb lit rim fragments with mu0 > 0 and mu clamped (LS ~ 2, f held at 150 deg) - white pixels ringing the
    eclipsed Moon, at fov 40 four at the corners of its bounding square. GPU check (section 11, MSAA 2x as the game): no
    pixel above 2 x the disc centre (16.4 x with the former function). The disc then follows K&S at every
    phase. R = 0.513 [D]: the full-phase disc of the texture's near side (projected-area mean albedo 0.2245 - the maria;
    the sphere's mean is 0.30) gives K&S's 0.313 lux (equivalent geometric albedo 0.115). Finding: the former disc
    (0.8 / 1.5 = 0.533) was only 1.04 x K&S at full phase (the review's 1.4 x used the sphere's mean albedo); its real
    error was the missing phase law - LS alone made the quarter Moon 4 x and a 30 deg crescent ~11 x too bright. Globals
    from `Atmosphere.update` (no "keep in sync" constant): `moon_regolith_scale` (R, from the tables' TOA
    illuminance), `moon_sun_scene` (DAY_EXPOSURE / d_AU^2: the Moon now follows the Sun's distance), `moon_earthshine`.
  - Moonlight: the Moon's illuminance outside the atmosphere by Krisciunas & Schaefer 1991 (PASP 103, 1033: Eq. 9
    m = -12.73 + 0.026 |a| + 4e-9 a^4, Eq. 8 I = 10^(-0.4 (m + 16.57)) fc; full Moon 0.313 lux at 384 400 km), x the
    opposition surge (above), x the distance squared, x the Earth's shadow at the Moon (lunar eclipse; no red glow)
    (`Atmosphere.moonlight_relative`; relative to the TOA Sun at 1 AU: full Moon 2.4e-6, x 1.35 at a = 0; its users
    multiply by the 1 AU tables / d_AU^2, so it follows the Sun's distance - the former d_AU^2 factor cancelled that,
    review M1). A second light of the solar spectrum through the same tables
    (`atm_moon_sky`, `atm_moon_at`, `atm_moon_sky_light`, `atm_moon_light`): the moonlit sky (sky.gdshader, the
    stars' background), the moonlit ground (terrain, Earth mesh, the model ground) and sea (a second pass of the sea
    model: the Moon's glitter path, the moonlit sky reflected), vehicles (world.gd `moon_light`, a second
    DirectionalLight without shadows, `Atmosphere.moon_light_at`), and the adaptation (the Moon's plate at the eye;
    in orbit the moonlit Earth in view). Active only while the Moon's plate is at least 0.1 % of the adaptation
    luminance (by day and at new Moon it is off: no change, no cost). Left out [call]: the Moon's reddening (B-V 0.92
    against the Sun's 0.65), the in-scattered moonlight between a ground point and the eye (the moonlit haze; the sky
    itself has it), the parallax of the Moon's direction (from the Earth's centre, up to 1 deg).
  - Values (render_check / GPU check, adapted, snapped; fix round 2026-10-25; the brightness term's B and S on top -
    renders: OLP-1 Sun -3 deg B 0.377 / S 0.675, -8 and below 0.368 / 0.521, totality 0.368 / 0.553, orbit sunset
    0.368 / 0.603, full Moon 0.368 / 0.521): pad noon 1.10 (no rods; OLP-1 noon
    view 1.123, the eye now 2 m above the surface); Sun -3 deg at the pad: L_a 90.8 cd/m^2 (half the glow), gain 464,
    no rods; Sun -8 deg: L_a 1.44, gain 5324, rod scale 0.025, zenith (0.0036, 0.0083, 0.025) - deep blue; Sun -12 deg:
    gain 11 235, rod scale 5.5; moonless night: L_a at the 1e-6 floor, gain 12 368 (the cone t.v.i. floor), rod scale
    138, magnitude-6 star visible; full Moon at the Cape (2024-10-17 05:00, near perigee, phase ~3 deg: moonlight
    2.84e-6 of the Sun with the surge): L_a 0.12 (the Moon's plate), gain 9977, rod scale 1.35, the GPU check's ground
    (albedo 0.3/0.19/0.10, phase 10 deg) 0.019 in scene units with 78 % of it from the rods, magnitude 2 visible, 5
    hidden; lunar night 2845; totality at Dallas (2 m above the terrain at 107 m): L_a 8.24 cd/m^2 = half the
    brightest ring sky, gain 2438, rods 0 (the zenith 0.99 and the ring ~16 cd/m^2 unchanged; the GPU check's eye at 2 m
    on the ellipsoid: 7.17, gain 2624) - a dark blue-grey sky over an orange horizon ring and a dark ground, no more
    "overcast day"; 400 km in the umbra 17.7 (as before); 400 km 60 s after orbital sunset (GPU check, Moon passed):
    limb glow 109 cd/m^2, gain 389, sunward limb peak 2.73 (< 4, not blown); render at the Cape, Sun -26.6 deg: glow
    23.3, gain 1325; orbit night under the full Moon, 400 km: L_a 0.168 (the Moon in view), gain 9429.
  - GPU check (section 13, 18 samples): day frames = the fixed exposure x the gain (0.02 %) and the gain = the former
    Weber gain L_ref / L_a within 1e-9 (noon, 400 km), no rods, no moonlight; display-referred quads (above); the Sun
    -8 deg zenith blue (blue > 0.004, was 0.01 before the brightness term: 0.0065 now); a moonless night's ground: a scene-referred quad at a starlit ground's luminance (0.002 lux
    [S: Schlyter, moonless clear night sky incl. airglow] x albedo 0.2 / pi = 1.27e-4 cd/m^2 - no airglow or starlight
    is modelled, so the sky itself is 0) renders as the model's cone + rod image (0.2 %) and below the display
    threshold 1.5 x 10^-1.255 (its scotopic luminance is below the rods' absolute threshold: dark), pixel luminance
    0.019 (0.0068 with B 0.368); the full-Moon ground: its cone image (the post step undone) = the CPU's moonlit
    surface within 1 % (albedo x the Moon's direct and sky light x its relative illuminance x T), rods 78 % of its
    luminance (0.0072 on screen with B); stars
    (magnitude 6 moonless, 2 / 5 under the full Moon); the earthshine on the 2026-11-12 crescent 16 % above the sky
    beside it. Section 10: the limb glow on the game's path (the Moon passed: chosen, > 0); section 11: the adapted
    totality takes the glow (chosen, > 1.5 x the zenith), a twilight eye off the shadow on the eclipse day keeps its
    glow (penumbra on the Earth, eye fraction 0: glow 90.8 cd/m^2 chosen).
  - Limits: one adaptation per view (no local / foveal adaptation: bright parts in a dim view still saturate - the
    full Earth from the lunar night, the full Moon's disc, city lights take the rods' tint); no loss of acuity, no glare model; the rods'
    slower dark adaptation (~30 min) is paced like the cones' (110 s); no airglow, zodiacal light or integrated
    starlight (a moonless sky is black, not ~2e-4 cd/m^2); Jensen's k on the adaptation luminance rather than per
    pixel; the brightness term's B and S are one value per view (CIECAM02 is photopic: held below 79 / 4 cd/m^2), no
    Bezold-Bruecke hue shift (the lunar day keeps B = S = 1: its eye adapts to the sunlit plate, 4.2e4 cd/m^2).
- **Solar eclipses.** While the Moon's penumbra touches the atmosphere (`Atmosphere.penumbra_touches`), the
  in-scattering along a ray is split into segments at d (i/n)^2 (8 for the sky, Moon, Sun, stars and the sea's
  reflected sky; ceil(d / 12.5 km) clamped to 1..8 for the aerial perspective [call, performance]), each
  T(x,p_i) S(p_i) - T(x,p_i+1) S(p_i+1) weighted by the Sun's visible fraction at its midpoint (`eclipse.gdshaderinc`).
  **Approximation (documented):** this weights multiple scattering into the umbra like single scattering, which alone
  leaves the umbra sky exactly black; a **second-order umbra source** adds the light scattered once more inside the
  umbra: J = F / (4 pi) per segment, F the eclipsed sky's fluence on the umbra axis at 0 / 4 / 12 km
  (`Atmosphere._eclipsed_sky_fluence`: zenith + rings at 30 / 11 / 6 / 3 / 1 deg, the lower hemisphere above the
  ground; once per simulated second on a worker; 25.5 / 59.0 / 150.6 cd/m^2 sr at 2024-04-08 18:42). Totality at
  Dallas: zenith 0.993 cd/m^2 (real totality skies ~1-2), the horizon 16 x the zenith (the 360-degree twilight glow),
  magnitude-1 stars visible, magnitude 4 hidden. The ground's sky light gets the same eclipse factor (zenith 0.413,
  four azimuths at 30 deg 0.130, four at 6 deg 0.0167; per vertex). The eclipsed Moon (GPU check, 2026-10-25): the
  full Earth's earthshine 8.49 lux lights the near side (regolith ~0.43 cd/m^2 at the mean albedo, x T); with the
  sky in front the disc centre is 1.51 cd/m^2 against 1.08 beside it (+40 %, readily visible as a slightly brighter
  disc). Physically consistent for a Moon seen through the sky without a corona; the real totality Moon looks dark
  because the inner corona (~1e-6 of the solar disc's luminance near the limb) surrounds it - the corona is
  not modelled (open). Limits: one shadow-axis fluence for the whole umbra; no refraction; the eye in the umbra
  adapts to the penumbral Earth it sees.
- **Validation.** Core `[atmosphere]` (21 ctest cases, incl. the aerial perspective to a point before the entry):
  Rayleigh coefficient and vertical optical depth vs Bucholtz 1995 (1 % / 2 %); transmittance vs a direct optical-depth
  integral (3.24e-3) and single scattering vs a brute-force
  raymarch (1.07e-3); off-texel lookups 7.1e-3, horizon seam 9.2e-3; multiple scattering 5 vs 7 orders 1.96e-3; direct
  sun at air mass 1.5 vs ASTM G-173 (400-700 nm) -2.1 %, runtime direct beam vs raymarch < 0.05 %; zenith at Sun 60 deg
  3819 cd/m^2 (B/R 1.97), horizontal illuminance at Sun zenith 125.4 klx, sky at 50 km 7.1e-4 x the ground, twilight
  zenith bluer than noon (B/R 3.12 at -4 deg); ellipsoid mapping < 1 mm; cache round trip / damage / stale hash /
  failed write. GPU `game/tests/atmosphere_check.gd` (HDR viewport, float readback, 2 % per channel unless stated;
  RGBAH and `--f32`): 86 samples - sky, sunset, limb from 400 km and from the Moon distance, the pad camera's float64
  altitude path, ground and terrain samples (3 %), stars / Sun / Moon, lit cubes (5 %), Sun colour, eye adaptation,
  eclipse, eye adaptation away from the Earth (lunar day, lunar night, the Earth term at lunar distance, deep space),
  earthshine, night vision (section 13); the run asserts its sample count (`EXPECTED_SAMPLES`: a GDScript error in an
  awaited sample ends it silently); worst sky deviation 0.52 % (the 50 km limb from the Moon distance, float32 rounding at 3.8e8 m), the Earth
  mesh samples -2.1 / -1.7 % (its facets lie up to ~0.5 km below the ellipsoid; 0.15 % against the point 300 m lower).
- **Performance** (`terrain_perf.gd`, RTX 4090, 2560 x 1440, gate p99 <= 8 / max <= 16 ms; Task 10). Sky cubemap
  incremental at 128 (realtime 256 cost VRAM, not time: the frame is CPU-bound), half-float scattering tables.
  Default epoch (no eclipse): p99 7.75 / 7.90 / 7.90, max 10.7 / 12.5 / 11.7 ms in three consecutive runs (GPU <= ~3 ms;
  Cape p99 ~9.1 while ~240 nodes load). The max-gate failures of Tasks 6-9 (single 15-50 ms "process" frames) were
  the run's own per-second console prints (fixed in terrain_perf.gd). A rarer stall remains: ~1 run in 3 shows one
  frame of 11-17 ms (once 17.4 ms, failing the gate; in the final fix wave once 33.7 ms at t = 2.98 s, the same
  signature, the next run PASS: p99 7.75, max 13.2) at a fixed instant of the descent (t = 3.06 s or 10.89 s), inside
  the RenderingServer draw but outside the viewport render (measured viewport CPU 0.3 ms, GPU 1.4-1.9 ms), no
  draw-time pipeline compilation, while a batch of 4 terrain builds runs on the worker pool. It depends on the pool:
  `threading/worker_pool/max_threads = 8` removed it in 9 of 9 runs but limits the terrain to 2 concurrent builds
  (the Cape loads in > 20 s instead of ~8 s); 10 threads with 7 low-priority slots kept it (3 of 8), 24 threads with 2
  slots made it worse (6 of 8, 23-31 ms). Not fixed (engine-internal; open). Eclipse epoch (2024-04-08 18:42, the
  segments active, Starbase 80 % partial): p99 8.7 ms, max 12-13 ms - GPU-bound (7-10 ms in the descent); the
  per-vertex sky-light eclipse factor costs ~3.5 ms of it (9 directions x 4 segments on ~8 M terrain vertices;
  measured by switching it off). Adaptive segment counts (by the spread of the Sun's visible fraction along a ray)
  were tried and gave no measurable gain (the rays at Starbase cross enough penumbra gradient), so they were not
  kept. Eclipse-only; a shadow-plane texture of the factor, updated per simulated second, would remove it (open).
  Night vision (2026-10-25): by day nothing is dispatched and no moonlight is evaluated; the default-epoch run gave
  p99 7.83 / max 24.5 ms (the known stall at t = 3 s, inside the draw: "rest" 23 ms) and on the rerun p99 7.67 / max
  13.3 ms (PASS). At a full-Moon night (--utc=2024-10-17T05:00:00: the rods' compute pass, the moonlit sky, ground
  and sea terms active): p99 7.88 / max 14.3 ms (PASS; GPU-side cost hidden by the CPU-bound frame).
  Brightness term (2026-10-25): by day B = S = 1 and the post step is not dispatched (no new cost on the default
  epoch). Its run (default epoch) gave p99 47.1 / max 56.3 ms (FAIL) with the VR game running beside it on the
  same GPU (99 % load; p50 13 ms against 4.6 before): not a measurement of the change, to be rerun on an idle GPU.
  The step now also runs for the chroma alone whenever L_a < 1e4 cd/m^2 (low Sun, dusk, night): one full-screen
  compute pass on frames that had none before.
- **Astronomical refraction (2026-10-25).** `core/environment/atmospheric_refraction`: n - 1 = K rho (Gladstone-Dale,
  Edlén 1966 standard air at 550 nm, 2.778e-4) from the US76 density on a 25 m grid to 120 km; a ray's bending by the
  Bouguer invariant and the zenith-angle integral (Auer & Standish 2000), also through a tangent point below the
  observer (orbit, limb). Validation: Laplace's theorem at 45 deg (57.16", 0.6 %: US76's lapse-rate breaks make the
  integrand jump, Simpson with 128 intervals then converges as O(step)), the horizon from sea level 33.0' (Bennett
  1982 scaled to 15 deg C / 1013 hPa: 34.0', 5 %; it depends on the lapse rate). `refraction_lift_table` inverts it to
  the lift over the TRUE elevation (64 samples, quadratic grid from the grazing ray; an orbit's folded image below
  ~12 km tangent height keeps the upper branch; empty beyond ~330 000 km where limb rays pass the nadir), ~1.15 ms.
  Game: `world/refraction.gd` rebuilds it when the camera's distance from the Earth's centre changes by > 0.5 % of its
  height (>= 5 m; ground views never, orbit rarely, ascent a few times per second), uploads an R32F texture + globals
  `refraction_*`; `shaders/refraction.gdshaderinc` turns each vertex of the Sun disc, the Moon and the stars up in its
  vertical plane (the geodetic vertical, every frame) - the disc rises ~0.57 deg at the horizon, flattens to ~0.85 and
  stays visible ~2-3 min after its geometric set; the fragments' sky / transmittance follow the apparent ray (an
  approximation of the curved path). Not refracted: the Sun light and the sky tables (they use the geometric Sun: the
  ground's direct light ends at the geometric set), terrestrial refraction (the horizon dip), dispersion (green
  flash). `render_check --no_refraction` / `--look_at_sun` compare.
- **Known limits.** No clouds, weather-dependent aerosols, airglow; one aerosol set for the whole Earth;
  one Sun light (above); the totality approximation (above); night vision's limits (above); the 256 x 128 Earth mesh
  in scaled space sits up to ~0.6 km below the ellipsoid (-2 % in its aerial perspective).

## atmosphere-scattering.md before static clouds Task 9b (2026-10-27): four tables

- Tables and cache: "... file `data/atmosphere/earth_atmosphere.bin` (83.9 MB: header with magic, version, input hash,
  sizes and parameters, four float32 arrays, FNV-1a 64 checksum verified before parsing ..." (format version 2,
  `atmosphere_tables_code_version` 3); GPU "scattering / Mie / irradiance RGBAH; ~50 MB VRAM". The single Rayleigh
  scattering was only inside the scattering table (with the multiple orders); Task 9b keeps it apart for the cloud
  shadows on the air.

## City lights (removed 2026-10-29, user: one Earth texture, night by lighting only)

Former model, verbatim from docs/subsystems/eye-adaptation.md:

  - City lights (user ruling 2026-10-25): scene-referred, L = `Atmosphere.CITY_LIGHTS_PEAK_CD_M2` x the night
    texture's sRGB-decoded texel squared, x T and the aerial perspective (global `earth_city_lights_cd_m2`;
    `earth.gdshader`, `terrain_earth.gdshader`; before the tables load the display value). Black Marble 2016 is an
    8-bit visualisation, so the scale is [E]: a saturated white texel (4.9 km) is 0.3 cd/m^2 upward luminance,
    anchored to lit roads (0.75-1.5 cd/m^2, EN 13201-2 M3-M2) and VIIRS DNB radiances of bright cores (~100-300
    nW/(cm^2 sr), ~0.1-0.4 cd/m^2, arXiv 2510.02977). Texture stays 8192 x 4096; follow-up: Black Marble VNP46A4 at
    15 arcsec near the sites, masked by the imagery's urban footprint.
