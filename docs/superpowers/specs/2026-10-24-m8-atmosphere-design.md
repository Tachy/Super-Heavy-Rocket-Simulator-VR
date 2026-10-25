# M8 atmosphere (sky and aerial perspective) - design

Date: 2026-10-24. Milestone order: the next M8 part after the terrain plans, the sea state and the mission panel.
The user's words: "Wichtig ist mir der fließende Übergang in den Weltraum je nach Sicht." and "leg einfach los mit
physikalischer Grundlage, dann wird alles stimmen, auch die Farben". Every decision below not quoted from the user is
Claude's call, marked **[call]**. The user's earlier note on M8 (master plan, "Notiz des Nutzers", 2026-10-12) is the
background: Rayleigh + Mie scattering, Bruneton or raymarching, the VR frame budget, the float64 camera-to-Earth vector,
no separate cloud-shell mesh.

## Intent

The sky is black today: the pad stands under a black sky, the Earth from orbit has no limb, the sea reflects a
placeholder blue and vehicles get a constant ambient colour. This part gives the Earth a physically computed
atmosphere: blue daytime sky, red sunsets, the white haze around the Sun, the darkening sky during ascent, the thin blue
limb seen from orbit and the Moon, the haze over distant terrain and vehicles - one model for every viewpoint, so the
transition from the pad into space is continuous and depends only on where the camera is, where it looks and where the
Sun is. Colours come out of the physics (spectral scattering converted to display colours), not from tuned constants.

## Goals

1. **One physical model, every viewpoint [call: Bruneton 2017 precomputed atmospheric scattering].** Eric Bruneton's
   "Precomputed Atmospheric Scattering: a New Implementation" (2017, BSD-3 licence, github.com/ebruneton/
   precomputed_atmospheric_scattering) - the improved form of Bruneton & Neyret 2008: transmittance, single and multiple
   scattering (Rayleigh, Mie, ozone absorption, ground bounce) precomputed into tables; the runtime looks the tables up
   (no per-pixel raymarching: the VR budget of the user's note). It is valid from the ground to outside the atmosphere,
   which is exactly the continuous transition. Chosen over Hillaire 2020 (per-frame LUTs, RGB only) because it computes
   spectrally and converts to colour, and because its precomputation can run in our double-precision core.
2. **Precomputation in the core (C++, float64), not on the GPU [call].** A port of Bruneton's precomputation
   (`functions.glsl` + `model.cc`; he ships a C++ reference build of the same functions) into `core/src/environment/`
   (new files, e.g. `atmosphere_optics.*` for the coefficients and `atmosphere_scattering.*` for the tables; module
   `environment` - optics of the Earth's atmosphere). Parallel over table cells (std threads). Output: the
   transmittance, scattering (Rayleigh, separate single Mie) and irradiance tables in **luminance** (Bruneton's
   precomputed-luminance mode: 15 wavelengths 360-830 nm folded with the CIE 1931 2-degree colour matching functions
   into XYZ, then linear sRGB with D65; **no white balance** [call]: the TOA Sun comes out slightly warm-white as it is).
   The same C++ code also evaluates the model on the CPU (sun transmittance at a point, sky irradiance, sky radiance in
   a direction) for the game's light values and for the tests.
3. **Physical inputs, data-driven** (`config/atmosphere/earth_optics.toml` [call], provenance tags):
   - Rayleigh: scattering coefficient per wavelength from the refractive index of air (Peck & Reeder 1972) and the
     number density, with the King factor (Bucholtz 1995) [S]; the vertical profile is the **US76 number density** of
     `environment::Us76Atmosphere` (not an exponential fit) [call].
   - Ozone: absorption cross sections (Serdyuchenko et al. 2014, as tabulated by Bruneton) [S] and the US standard ozone
     profile, column ~300 DU [S/E].
   - Aerosols (Mie): a clear-day continental / maritime mix: optical depth ~0.1 at 550 nm, Angstrom exponent,
     single-scattering albedo ~0.9, scale height ~1.2 km, Cornette-Shanks phase with g ~0.8 [E] - one parameter set
     for the whole Earth (no weather yet).
   - Solar spectrum at the top of the atmosphere: ASTM E-490 AM0 (NREL) [S], scaled by the true Sun-Earth distance of
     the epoch at runtime (the tables are for 1 AU).
   - Ground albedo for the multiple-scattering bounce: constant ~0.1 [E] (no clouds yet).
   - Atmosphere top ~100 km [call] (the limb glow from orbit needs it; Bruneton's 60 km is for ground views).
   Constants and small tables (CIE CMFs, ozone cross sections, solar spectrum samples) live with their sources as
   `constants.hpp` requires (a data header under `core/include/starsim/environment/` with the citation is fine).
4. **WGS84 instead of a sphere [call].** Bruneton's model is spherical. The atmosphere is evaluated in a space where the
   WGS84 ellipsoid becomes a sphere of radius a (polar axis stretched by a/b = 1.00336): the ground matches the
   ellipsoid exactly everywhere, optical paths are off by at most 0.34 %. CPU and shaders use the same mapping. The
   model's ground radius sits slightly below the lowest surface that matters (pads are at about -21 m ellipsoidal,
   the geoid reaches about -106 m) [call: builder picks, documented]; cameras and points below it are clamped onto it.
5. **Table cache.** The core writes the tables to `data/atmosphere/` (binary, float32, a header with the table sizes
   and a hash of the inputs and the code version); the GDExtension loads them, or computes and writes them when
   missing or stale. `tools/package.ps1` generates them before packaging so installed games start without the wait.
   A first-run computation shows a German status text; target well under a minute on the 24-core machine [E].
6. **Runtime (Godot), one shader include** `game/shaders/atmosphere.gdshaderinc` [call]: the port of Bruneton's runtime
   functions (`GetSkyRadiance`, `GetSkyRadianceToPoint`, `GetSunAndSkyIrradiance`, transmittance lookups) on 2D / 3D
   float textures, with the ellipsoid mapping. Used by:
   - **Sky**: a Godot sky shader (`Environment.BG_SKY`) draws the sky for every pixel without geometry, from the
     camera's true position. In the radiance-cubemap pass it also draws the Earth's ground (day texture lit by the
     model, seen through the atmosphere), so Godot's ambient and reflections for standard materials carry sky light,
     ground bounce on the pad and earthshine in orbit from the same model; the constant ambient colour goes.
   - **Earth surfaces** (`earth.gdshader` incl. scaled space, `terrain_earth.gdshader`, `water.gdshaderinc`): ground
     lighting by the Sun transmitted to that point plus the sky irradiance on its normal; then aerial perspective
     between camera and point (transmittance + in-scattering); the sea's reflected sky is the model's sky radiance in
     the reflection direction (replaces `sky_radiance`). City lights are seen through the same transmittance.
   - **Moon, Sun disk, stars**: seen through the atmosphere when the view ray crosses it (transmittance; the opaque
     Moon also gets the in-scattered sky added, so the daytime Moon is pale). The Sun disk reddens and dims at the
     horizon. Stars: their brightness is attenuated and a star is hidden when its contrast over the sky background
     falls below the eye's detection threshold (a Schaefer / Garstang-type threshold over background luminance [S/E]) -
     stars fade out in twilight and appear above the atmosphere, without a switch.
   - **Vehicles, tower, pad, drone ship** (today StandardMaterial3D): a shared shader (or material conversion) that
     keeps their current look and adds aerial perspective through Godot's `FOG` output, so a rocket climbing away from
     a ground camera fades into the sky. Additive plumes / trails: transmittance only [call].
   - **Sun light for vehicles**: the DirectionalLight's colour and energy follow the Sun transmitted to the followed
     body (the core's CPU evaluation; red and dim at sunset, zero below the horizon), together with the existing eclipse
     fraction.
7. **One exposure for everything [call].** Today a white diffuse surface under the full TOA Sun renders as
   `day_exposure` (1.5). The luminance tables are scaled by the same factor (scene value = luminance x
   `day_exposure` x pi / E_v,TOA), so sky, ground, sea and vehicles share one camera exposure; vehicles' light energy is
   brought onto the same scale (a white Lambertian under the TOA Sun gives `day_exposure` in every material).
   **Eye adaptation [call, 2026-10-25]** (now in scope, reversing the earlier "no automatic exposure"): the fixed day
   exposure renders civil twilight black (CPU zenith luminance 17 cd/m^2 at Sun -3 deg, 0.48 at -6 deg against
   ~1600-8000 by day), which contradicts the seamless, physically coloured transition. One exposure gain multiplies
   every physical luminance of the scene (the tables' luminance scale, the Sun light, Moon, Sun disc, plumes; stars
   and city lights stay display-referred): Weber adaptation (Ferwerda et al. 1996) to the luminance of a white
   Lambertian at the eye under the transmitted Sun and the sky (CPU), relative to the TOA Sun - gain 1 in orbit by
   day -, bounded to 25 (the noon / sunset zenith ratio: twilight never brighter than day, nautical twilight dark),
   paced like the eye (light adaptation 0.5 s, dark adaptation 110 s after Pattanaik et al. 2000; snapped on a view
   change). Exposure only: no white balance, no colour change. Fidelity flag `Atmosphere.eye_adaptation_enabled`
   (on in the game, off in the GPU check's comparisons). Works in VR as on the desktop (shader uniforms, no
   post-process).
   **Night vision - user decision 2026-10-25 ("like the eye"; amends the paragraph above).** The cap 25 was a display
   call: it rendered the twilight sky at Sun -8 deg and the earthshine on the Moon (~0.5 cd/m^2, physically brighter
   than that sky) black. Replaced by a perceptual mapping, Ferwerda et al. 1996 [call: chosen over Pattanaik et al. 2000
   and Kirk & O'Brien 2011] - threshold matching with separate threshold-versus-intensity functions for cones
   (photopic, their Eq. 4) and rods (scotopic, Eq. 5), combined over the mesopic range (Eq. 6, display = cones + k x
   rods, k from 1 to 0 over 0.01-4 cd/m^2 with the smooth step of Jensen et al. 2000, "Night rendering"). Above ~80
   cd/m^2 the cone threshold is Weber's law, so the day look is unchanged (the cone gain equals the former Weber gain
   L_ref / L_a); below it the cone threshold flattens and the rods take over - Weber's law fails at scotopic levels, so
   a moonless night stays dark. The cone gain multiplies every physical luminance in the shaders (as the former gain);
   the rods' achromatic image is added by one full-screen post step (a CompositorEffect after the transparent pass,
   before glow and the tone mapper; per pixel, the same for both VR eyes): rgb + k g_r / g_c x max(w_s . rgb, 0) x
   tint, the scotopic luminance from the CIE 1951 V'(lambda) projected onto linear sRGB, the rod image tinted
   blue at (x, y) = (0.25, 0.25) (Jensen et al. 2000; the common night-rendering convention). Display-referred emitters
   (plumes, city lights, the stars' own brightness, UI markers, the VR panel) take the inverse of that step in their
   shaders and look as before [ruling: city lights and plumes stay display-referred - bright sources the fovea sees in
   colour; the global mapping without local adaptation would grey them out, and the night-light texture has no
   radiometric calibration; amended by the user ruling in the fix round of 2026-10-25: city lights are scene-referred
   with an [E] luminance scale and go through the eye model]. The adaptation luminance gains a scotopic twin (the same view-independent plates, their
   colour through w_s) and a twilight term: with the Sun below the horizon (or behind the Earth, in orbit) the
   brightest sky (the glow toward the Sun) - Ferwerda's "highest visible luminance" made view-independent. The
   Moon becomes a light (Krisciunas & Schaefer 1991 phase law): the moonlit sky, the moonlit ground and sea, a second
   light on vehicles, and the Moon's plate in the adaptation - only while it can show (its plate >= 0.1 % of the
   adaptation). The cap remains only as a numeric bound (1e5; the t.v.i. floor bounds the cone gain at ~1.2e4); the
   full Earth from the lunar night still saturates (expected). Time constants, snaps and the star threshold unchanged.
8. **Solar eclipses in the sky [call].** Inside the Moon's shadow the sky must darken (the default render check is the
   2024-04-08 eclipse). When the penumbra touches the Earth (CPU check), the in-scattering along a view ray is split
   into a few segments (Bruneton's difference form S(x) - T(x, y) S(y)), each weighted by the Sun's visible fraction
   (`eclipse.gdshaderinc`) at its midpoint: the umbra sky goes dark while light scattered from outside the shadow gives
   the 360-degree twilight horizon of totality. A documented approximation (multiple scattering into the umbra is
   weighted like single scattering).
9. **Numerics and the floating origin** (the user's pitfall): the shaders get the camera-to-Earth-centre vector and the
   camera's altitude differenced in float64 on the CPU (as `earth_terrain_camera_hi/lo` already do for the terrain);
   the near-ground parameters (Bruneton's rho = sqrt(r^2 - R^2)) are formed from altitude and (r + R), never from the
   difference of two large squares in float32, so the horizon neither jitters nor bands at the pad or in orbit.

## Non-goals

- Clouds (the next M8 part: GIBS / GOES data, cloud shell, volumetric clouds below ~20 km) - but the include is laid
  out so clouds can later add their own shadowing and light.
- Weather-dependent aerosols, humidity, water-vapour absorption, airglow, aurora, noctilucent clouds.
- Atmospheric refraction (flattened Sun at the horizon, astronomical refraction of the stars), scintillation.
- ~~Automatic exposure / eye adaptation~~ - now in scope [call, 2026-10-25], see Goal 7: civil twilight rendered black
  at the fixed exposure. ~~Mesopic / scotopic vision~~ - in scope by the user decision of 2026-10-25 (Goal 7, night
  vision). Still out: glare and bloom models, the loss of acuity at night, local (foveal) adaptation, the rods' slower
  dark adaptation, airglow / starlight / zodiacal light as night-sky sources.
- Light shafts / volumetric shadows of mountains, vehicles or the tower in the air.
- Any change to flight physics: the US76 atmosphere stays the only atmosphere the simulation flies through; this part
  only reads it.
- Moon terrain cast shadows (an open terrain point, not atmosphere).

## Anti-goals

- Tuned colours: no hand-picked sky gradients, sunset tints or ambient colours that the model does not produce; a
  wrong colour is fixed in the inputs or the code, never by a colour fudge.
- Visible switches: no altitude where the sky, the limb or the haze swaps model, pops or blends between two
  techniques; the scaled-space Earth and the terrain quadtree show the same atmosphere at their hand-over.
- Breaking the VR budget: the terrain milestone's performance gate (terrain_perf.gd, 1440p desktop proxy p99 <= 8 ms,
  max <= 16 ms) must still pass with the atmosphere on.
- Horizon jitter or banding from float32 (the floating-origin pitfall).
- Physics in GDScript: the precomputation lives in the core; GDScript only loads tables and sets uniforms.

## Validation (documented tolerances)

Core (`[atmosphere]` tag [call], unit tests + references):
- Rayleigh scattering coefficient at sea level vs Bucholtz 1995's tables (e.g. 400 / 550 / 700 nm) within 1 %;
  vertical Rayleigh optical depth vs Bucholtz within 2 %.
- Transmittance table vs direct numerical integration of the optical depth (zenith and slant rays, several altitudes)
  within the table's interpolation error (documented, ~0.5 %).
- Direct normal solar irradiance at sea level for an air mass of 1.5 vs ASTM G173 (direct, 400-700 nm band) within a
  tolerance that reflects the different aerosol / water-vapour assumptions (~5-10 %, stated with the reason).
- Port check: the tables vs an independent brute-force integrator in the tests (transmittance within 0.5 %, single
  scattering within 1 %), multiple scattering converged (orders 4 vs 6 within 1 %). (Plan amendment: Bruneton's
  repository has no tabulated reference values to compare against.)
- Physical plausibility: zenith sky luminance for a high Sun in the range of measured clear skies (a few thousand
  cd/m^2), the horizontal illuminance for a Sun at zenith ~100 klx, the sky at 50 / 100 km at least 2 / 4 orders of
  magnitude darker than at the ground, the twilight sky at the zenith bluer than at noon (ozone).
- Ellipsoid mapping: a point on the WGS84 surface maps onto the model's ground radius at every latitude (1 mm).
- Cache: hash change recomputes; a damaged file is recomputed, never read.

Game (GPU, render_check.gd / a GPU check script):
- Shader vs core: the shader's sky radiance and aerial perspective at sample pixels agree with the core's CPU
  evaluation within 2 % (guards the port and the uniforms).
- Visual set read as PNGs: OLP-1 at noon and at sunset (looking at the Sun and away), ascent views at 10 / 30 / 60 /
  100 km, the ISS orbit looking at the limb (day side and terminator), the Earth from the Moon, the 2024-04-08 totality
  from the ground, the Cape coast with the drone ship from 5 km (sea reflection), a rocket seen from the pad at 40 km
  (haze).
- Performance: terrain_perf.gd gate still passes; VR check by the user (Start-VR.cmd: pad, ascent, orbit).

## Left to the builder

Table sizes (Bruneton's 256 x 128 x 32 scattering, 256 x 64 transmittance, 64 x 16 irradiance as the start; more r
samples if the 100 km top needs them), float32 vs float16 textures (float32 unless memory or speed say otherwise),
the scattering orders (4+), how the vehicles' materials get the `FOG` output (one shared shader vs per-file materials),
the sky cubemap update mode (realtime / incremental) under the performance gate, the number of eclipse segments, the
star threshold's exact formula with its citation, the file format of the cache, and how the extension exposes tables
and CPU evaluations (`StarsimCore` methods). Licence and credit entries for Bruneton's code and the data tables
(`tools/data_pipeline/sources.toml` / `CREDITS.md`). Docs: a new "Atmosphere rendering" section in `docs/subsystems.md`,
the CLAUDE.md known-issues lines about the black sky / sea sky term / horizon glow updated, `docs/ansichten.md` where
views change.
