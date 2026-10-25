# Atmosphere: stars, solar eclipses, refraction (M8)

History and the reasons behind superseded decisions: `docs/history/atmosphere-rendering.md`.

Plan `docs/superpowers/archive/plans/2026-10-24-m8-atmosphere.md`, spec
`docs/superpowers/archive/specs/2026-10-24-m8-atmosphere-design.md`.
Part of the atmosphere docs; index `docs/subsystems/atmosphere-rendering.md`. Eye model behind the star
threshold:
`eye-adaptation.md`.

## Model

- **Stars** (`star_visibility.gdshaderinc`): x T and hidden below the eye's detection threshold over the background,
  I_th = c1 (1 + sqrt(c2 B))^2 (Schaefer 1990, PASP 102, 212; smooth step +-0.5 mag [E]); limiting magnitude 7.9 at
  B = 0, about -4 at the noon sky. B = max(the sky behind the star, the luminance the eye is adapted to)
  (`EyeAdaptation.adaptation_cd_m2`, light 0.5 s, dark 110 s, log space, unclamped, floor 1e-6 cd/m^2 [call], global
  `atmosphere_adaptation_cd_m2`, 0 with the adaptation off). Sunlit space views are starless (day-side orbit: L_a
  4.2e4 cd/m^2 at 400 km, limiting magnitude about -6; GPU check: magnitude 0 and 4 hidden beside the limb); stars show
  on the night side, in the Earth's shadow, in twilight and in totality. Moon and Sun disc need no threshold; planets
  are not drawn. At night the moonlit sky behind a star counts too.
- **Solar eclipses.** While the Moon's penumbra touches the atmosphere (`Atmosphere.penumbra_touches`), the
  in-scattering along a ray is split into segments at d (i/n)^2 (8 for the sky, Moon, Sun, stars and the sea's
  reflected sky; ceil(d / 12.5 km) clamped to 1..8 for the aerial perspective [call, performance]), each
  T(x,p_i) S(p_i) - T(x,p_i+1) S(p_i+1) weighted by the Sun's visible fraction at its midpoint (`eclipse.gdshaderinc`,
  mirrors `core/src/environment/eclipse.cpp`). Approximation: this weights multiple scattering into the umbra like
  single scattering (umbra sky exactly black alone); a second-order umbra source adds J = F / (4 pi) per segment, F the
  eclipsed sky's fluence on the umbra axis at 0 / 4 / 12 km (`Atmosphere._eclipsed_sky_fluence`: zenith + rings at 30 /
  11 / 6 / 3 / 1 deg; once per simulated second on a worker; 25.5 / 59.0 / 150.6 cd/m^2 sr at 2024-04-08 18:42).
  Totality at Dallas: zenith 0.993 cd/m^2, the horizon 16 x the zenith, magnitude-1 stars visible, magnitude 4 hidden.
  The ground's sky light gets the same eclipse factor (zenith 0.413, four azimuths at 30 deg 0.130, four at 6 deg
  0.0167; per vertex). Eclipsed Moon: full Earth's earthshine 8.49 lux; with the sky in front the disc centre is 1.51
  cd/m^2 against 1.08 beside it (+40 %); the inner corona (~1e-6 of the solar disc's luminance) is not modelled. Limits:
  one shadow-axis fluence for the whole umbra; no refraction; the eye in the umbra adapts to the penumbral Earth.
- **Astronomical refraction.** `core/environment/atmospheric_refraction`: n - 1 = K rho (Gladstone-Dale, Edlén 1966
  standard air at 550 nm, 2.778e-4) from the US76 density on a 25 m grid to 120 km; a ray's bending by the Bouguer
  invariant and the zenith-angle integral (Auer & Standish 2000), also through a tangent point below the observer.
  `refraction_lift_table` inverts it to the lift over the TRUE elevation (64 samples, quadratic grid from the grazing
  ray; an orbit's folded image below ~12 km tangent height keeps the upper branch; empty beyond ~330 000 km), ~1.15 ms.
  Game: `world/refraction.gd` rebuilds it when the camera's distance from the Earth's centre changes by > 0.5 % of its
  height (>= 5 m), uploads an R32F texture + globals `refraction_*`; `shaders/refraction.gdshaderinc` turns each vertex
  of the Sun disc, the Moon and the stars up in its vertical plane (geodetic vertical, every frame) - the disc rises
  ~0.57 deg at the horizon, flattens to ~0.85 and stays visible ~2-3 min after its geometric set. Not refracted: the
  Sun light and the sky tables (geometric Sun), terrestrial refraction (horizon dip), dispersion (green flash).


## Validation

- Refraction: Laplace's theorem at 45 deg (57.16", 0.6 %), the horizon from sea level 33.0' (Bennett 1982 scaled to
  15 deg C / 1013 hPa: 34.0', 5 %; depends on the lapse rate).

## Open points and known limits

- Eclipsed-Moon corona not modelled; no per-pixel / foveal adaptation.
