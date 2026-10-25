# Task 4 report: cloud optics, coverage-preserving noise, CPU cloud light

Status: **NEEDS_CONTEXT** (2 rulings). Everything is implemented, but NOT committed: 36 of the 38 `[weather]` cases pass.
Two brief assertions fail for physical reasons. As instructed, I did not tune constants to make them pass.

## Implemented
- `core/include/starsim/environment/cloud_optics.hpp` + `core/src/environment/cloud_optics.cpp`
  - `cloud_extinction_per_m`: 3 W / (2 rho r_eff), liquid + ice (rho_w 1000, rho_ice 917, r_l 10 um, r_i 30 um).
  - Phase function: Jendersie & d'Eon 2023, the approximate Mie function (HG + Draine mixture) for d = 2 r_l = 20 um.
    Its parameters: g_hg 0.99461, g_d 0.59379, alpha 27.114, w 0.49816. `cloud_phase_parameters(d)` exposes the
    fit, and `cloud_asymmetry_g()` gives g in closed form: **0.8793**.
  - Two-stream reference: delta-Eddington (Joseph et al. 1976), conservative closed forms of Meador & Weaver 1980.
    `two_stream_reflectance` and `two_stream_diffuse_transmittance` are for diffuse incidence, as the brief's
    signatures have no mu0. **Added** `two_stream_beam_reflectance(tau, g, mu0)`, because the slab test compares at two
    Sun zenith angles.
  - Multiple-scattering approximation (what the shader ports, closed forms):
    - exact single scattering of the direct Sun;
    - plus the delta-Eddington diffuse field (I0 + g' u_z I1) of the local plane-parallel column, i.e. the
      independent-column approximation (Bouthors 2008 idea, with two-stream in place of Monte Carlo tables);
    - plus the forward-peak-kept beam scattered once at wide angles (TMS-style split, Nakajima & Tanaka 1988).

    `cloud_multiple_scattering_source(...)` and `cloud_transport_scale()` (1 - g^2) carry it.
    `slab_reflectance(tau, mu0)` is the model's plane-parallel albedo: depth integrals are analytic, the hemisphere
    uses Gauss-Legendre 64 x 64. The header holds every formula and citation for Task 8.
    Shader inputs it needs:
    - Sun optical depth (light march);
    - vertical depth below the column top and the column optical depth (from the field);
    - two view transmittances: tau for J_ss, (1 - f) tau for J_ms.
  - CPU light:
    - `cloud_sun_transmittance`: Beer-Lambert ray march in ITRF. Midpoint steps of <= 25 m height and <= 1 km
      length; above 20 km it skips exactly. It stops when it climbs above 20 km (height along a line is convex) or
      when it enters the solid Earth (-2 km).
    - `cloud_sky_factor`: two-stream diffuse T of the column above.
    - `cloud_column_optical_depth_above`: exact trapezoids between layer bottoms and centres.
- `core/include/starsim/environment/cloud_noise.hpp` + `core/src/environment/cloud_noise.cpp`
  - Tileable Perlin (improved, 12 gradients) and Worley (wrapping cells, one point per cell). Integer hash
    (lowbias32), so results are deterministic on every platform.
  - Shape texture, 128^3 RGBA8:
    - R: Perlin-Worley (Schneider / Hillaire remap), rank-equalised to a uniform byte distribution. This is a monotone
      remap: the shapes stay the same and the coverage resolution is 1/256.
    - G, B, A: Worley fBm with 4, 8, 16 cells.
  - Detail texture, 32^3: Worley fBm with 2, 4, 8 cells.
  - `coverage_threshold[i]` comes from the R histogram (nearest texel count).
  - **Generation: 0.36 s** (22 threads), parallel over z slices.
- `core/src/environment/parallel_rows.hpp`: Task 3's `for_each_row` (hardware_concurrency - 2, at least 1, at most 32)
  is now in a shared internal header. `weather_snapshot.cpp` uses it (no duplication).
- `constants.hpp`: liquid water and ice densities, droplet and ice effective radii, with sources.
- `tools/validation/cloud_slab_monte_carlo.py`: an independent Monte Carlo slab with the same phase function, used to
  check the two-stream reference. Offline tool, fixed seed.
- Tests: `core/tests/test_cloud_optics.cpp` (6 cases, `[weather]`); CMake entries.

## Tests
Command: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[weather]"`
- RED (stubs returning 0): 5 of 6 new cases failed. Extinction, phase, overcast, noise and Sun transmittance failed.
  The slab case passed trivially (0 vs 0). Log: `build/t4_red.log`.
- Now: **36/38 pass**. Extinction, phase function (normalisation 1 within 1e-3, g 0.8793), noise coverage + tiling +
  determinism, and Sun transmittance (below / inside / above the slab, a Sun below the horizon crossing the slab twice,
  the sky factor, poles and date line) all pass. The noise case takes 0.72 s including two generations. Failing:
  "Slab reflectance agrees with two-stream" and "An overcast deck dims the ground physically". Log:
  `build/t4_green.log`.

## Ruling 1: slab reflectance within 15 % is not met (no approximation I found meets it without fitted constants)
Reflectance R for tau 1 / 3 / 10 / 30 / 50:

| Case | MC (4e5 photons) | delta-Eddington | Model (this task) | Wrenninge octaves a=b=c=0.5, 8 oct |
|---|---|---|---|---|
| mu0 = 1 | .0341 .1117 .3577 .6623 .7723 | .0365 .1165 .3575 .6636 .7738 | .0269 .0871 .3038 .6063 .7165 | .126 .153 .155 .155 .155 |
| model vs dEdd, mu0 = 1 | | | -26.3 -25.3 -15.0 -8.6 -7.4 % | +246 +31 -57 -77 -80 % |
| mu0 = 0.5 | .1374 .3174 .5624 .7717 .8452 | .1248 .2867 .5400 .7645 .8417 | .0825 .2138 .4458 .6677 .7448 | .234 .263 .264 .264 .264 |
| model vs dEdd, mu0 = 0.5 | | | -33.9 -25.4 -17.4 -12.7 -11.5 % | +88 -8 -51 -65 -69 % |

- **The reference is good.** delta-Eddington is within 7 % (mu0 = 1) and 10 % (mu0 = 0.5) of the Monte Carlo truth,
  and within 1 % for tau >= 30.
- **Octaves cannot work here.** A single-scatter octave sum only sees the top ~1 optical depth along the view ray, so
  it saturates at about 0.15-0.26 for thick clouds (true value 0.77-0.85). The same parameters overestimate thin
  slabs. Every choice of a / b / c only trades one end against the other: matching would be a fit.
- **Why my model falls short: it integrates sources along view rays.** Any volumetric shader does the same. With an
  Eddington source this is not flux-consistent with the two-stream fluxes: even pure delta-Eddington (Eddington phase,
  no exact single scattering) loses 7-25 %. A conservative semi-infinite isotropic slab gives 1.25 - (1 - ln 2) = 0.943
  instead of 1, analytically. Thin clouds are low because at tau 1 two thirds of the true reflection is multiple
  forward-then-back scattering, and the Eddington diffuse field represents that poorly.
- **Options for the controller:**
  - (a) Keep this model and set a documented tolerance from the measured errors. Worst case is -34 % (tau 1, 60 deg);
    for tau >= 10 it is within 17.4 % of delta-Eddington and 21 % of Monte Carlo. Cost: one test constant. Thin clouds
    are rendered up to ~1/3 too dark.
  - (b) Pure delta-Eddington source with the Eddington phase function. Errors -25 / -15 / -7 % at mu0 = 1 and
    -14 / -12 / -8 % at mu0 = 0.5 (tau 1 / 10 / 50; still > 15 % at tau 1-3, mu0 = 1). The exact Mie phase function
    disappears from the look: no silver lining or forward glow.
  - (c) Octaves as the brief's example: rejected on the numbers above.
  - (d) A Monte-Carlo-derived transfer table (Bouthors 2008 style, generated by the committed MC tool). Physically
    founded and not a fudge, but it is a texture lookup rather than a closed form, and it is new scope.

  My recommendation is (a).

## Ruling 2: overcast transmittance at tau 20 is 0.356, outside the brief's 0.1-0.3
- `two_stream_diffuse_transmittance(20, 0.8793)` = 1 / (1 + 0.75 x 0.1207 x 20) = **0.356**. The Monte Carlo truth
  with the same phase function is **0.346**. Asymptotic diffusion theory gives 4 / (3 ((1 - g) tau + 1.42)) = 0.348.
  So the model is right, and the brief's range does not match tau 20 for g ~0.88. Even g = 0.85 gives 0.31.
- The measured overcast mean of Kasten & Czeplak 1980 (G(8/8) / G(0) = 0.25, Hamburg) corresponds to tau ~33 in this
  model. The brief's upper bound of 0.3 needs tau >= 26.
- **Options for the controller:**
  - test tau 30 against 0.1-0.3 (gives 0.27);
  - or keep tau 20 with a physically derived range, e.g. 0.3-0.4 citing the Monte Carlo / diffusion value.

  No model change is suggested.

## Other deviations / additions (interface)
- `two_stream_beam_reflectance` added (see above).
- Also added, as the shader port's surface: `CloudPhaseParameters` / `cloud_phase_parameters`, `cloud_asymmetry_g`,
  `cloud_transport_scale`, `cloud_multiple_scattering_source`, `cloud_column_optical_depth_above`.
- `tileable_perlin` / `tileable_worley` are public, so the test can check exact periodicity.
- The shape channel the coverage rule applies to is R. It is rank-equalised, so t(cf) is ~1 - cf, but it is always
  read from the table.
- Ice uses the droplet phase function. This is a documented simplification.
- Grazing case: a Sun 5.5 deg below the horizon seen from 30 km (the ray dips to ~0.5 km). At 93 deg the ray never
  reaches the clouds.

## Self-review
- The C++ slab numbers reproduce the Python prototype exactly. Noise is deterministic (generated twice, compared).
- At the seam, the step across the wrap is 0.67-1.5x an interior step, on all channels and axes.
- No new external dependency. Constants carry citations. Fit coefficients are named in the .cpp with the citation in
  the header.
- Two header comment lines are 121 characters (repo has no clang-format; existing files have longer lines).
- Prototypes (octave / model / Monte Carlo) are in the scratchpad:
  `C:\Users\johan\AppData\Local\Temp\claude\C--Users-johan-Documents-KI-Projekte-Super-Heavy-Rocket-Simulator-VR\86d89457-57e6-4d50-8d88-8009dec6af83\scratchpad\proto.py`
  and `proto2.py`.

## After the rulings
Change the two test assertions (and the slab model, if (b) or (d) is chosen), re-run `[weather]`, then commit
`feat(environment): cloud optics, coverage-preserving noise and the CPU cloud light` and push.

## Rulings applied
- Ruling 1: model unchanged; test "Slab reflectance agrees with two-stream" now asserts tau >= 10 within 22 %, tau 1 / 3 within 36 % (measured errors, MC check cited, Bouthors 2008 transfer table recorded as follow-up).
- Ruling 2: "An overcast deck dims the ground physically" tests tau 30 against 0.1-0.3 (Kasten & Czeplak 1980, 0.25) and tau 20 against the Monte Carlo 0.346 within 5 % (tools/validation/cloud_slab_monte_carlo.py). Energy-conservation check (R + T = 1) moved to tau 30 with the same t.
- Command: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[weather]"`
- Output: `100% tests passed out of 38` (first run had one failure from a mismatched tau in the R + T check, fixed).
