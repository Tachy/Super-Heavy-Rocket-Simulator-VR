# Moon procedural near-ground surface

Spec `docs/superpowers/specs/2026-11-04-moon-procedural-surface-design.md`, plan
`docs/superpowers/plans/2026-11-04-moon-procedural-surface.md`. Current state after Task 1 (statistics and config).

## Model
Measured statistics only so far (`config/moon_surface.toml`); the generator (Task 2-3) builds on them.
- Small craters: equilibrium population, cumulative N(>D) = 0.0336 D^-2 per m^2 (2.2 % of geometric saturation),
  0.01 m - 2 km; 70 % degraded (d/D 0.07), 30 % fresh (d/D 0.13, rim height 0.024 D).
- Regolith roughness (what is not a crater or rock): self-affine, RMS height in a window W = 4 mm (W / 0.125 m)^0.78
  (1 m: 2.0 cm, 20 m: 0.21 m, 100 m: 0.74 m).
- Rocks: N(>D) = 1.54e-3 D^-2.286 per m^2, 0.01 - 10 m (N(>1 cm) = 57 per m^2, N(>10 cm) = 0.30 per m^2,
  N(>1 m) = 1.5 per 1000 m^2, N(>10 m) = 8 per km^2; area covered 3.1 % for 0.01-10 m, 0.68 % for 0.5-10 m). Axis
  ratios b/a 0.71, c/a 0.50 (sd 0.15), burial 0.3 +- 0.15 of the height, single-scattering albedo 1.35 x regolith.
  Local abundance: Diviner rock abundance (60 S - 60 N) divided by the 0.68 % the law implies; uniform elsewhere.
- Hapke (643 nm, highland proxy): w 0.429, b 0.220, c 0.514, B_S0 1.61, h_S 0.054, B_C0 0, h_C 1, theta_bar
  23.657 deg, K 1.

## Code
- `core/include/starsim/config/moon_surface_config.hpp`, `core/src/config/moon_surface_config.cpp`:
  `config::load_moon_surface_config(file)` -> `MoonSurfaceConfig` (craters, roughness, rocks, hapke);
  `SizeFrequency::cumulative_density_per_m2(D)` (forms `power` | `exponential`, not clipped to [min, max]).
  Missing / bad keys throw `ConfigError` with a German message naming the full key (`roughness.hurst_exponent`).

## Standing decisions
- Equilibrium, not the Neukum production function, for the small craters: every surface older than ~1 Gy is at
  equilibrium below the crossover diameter (hundreds of m on maria, km on highlands); the production function lies above.
- One power law per population (the interface holds one `SizeFrequency`): the rock law is the Surveyor cm-dm
  fit extrapolated to 10 m (see Sources for the join).
- Hapke parameters stay the self-consistent Sato et al. (2014) set (K = 1, B_C0 = 0, theta_bar fixed): mixing in a
  porosity K ~ 1.3 would need a refit of w.

## Validation
- `[moon_surface][config]`: the shipped file loads, every field finite, 0.5 < H < 1, 0 < w < 1, rocks down to
  <= 0.01 m, both cumulative laws strictly decreasing over their ranges; a missing key is named in German.

## Sources
Each value with tag, reference and the scale range it covers. Tags: [S] sourced, [E] estimate, [D] derived.

### Small craters
| Value | Tag | Source | Range |
|---|---|---|---|
| N(>D) = 0.0336 D^-2 / m^2 (2.2 % geom. saturation) | [S] | Minton et al. 2019 (Icarus; arXiv:1902.07746) sec. 1.2, Fit 1 to Fassett's Apollo 15 counts (LROC NAC 0.5 m/px): N(>r) = 0.0084 r^-2; geometric saturation 0.385 r^-2 (Gault 1970); in diameter x 4 | fitted 10-50 m |
| slope 2 | [S] | Xiao & Werner 2015 (JGR Planets 120): slope ~2 on all terrains, equilibrium 0.69-3.9 % of geometric saturation where it occurs below 500 m; Gault 1970: 1-10 %; Hartmann 1984: slope 1.83 | m - km |
| min 0.01 m | [E] | finest drawn scale; Shoemaker & Morris 1970 (Radio Science 5): steady state F = 10^10.9 c^-2 at all five Surveyor sites; Surveyor 7: 13 cm - 3 m follows it | extrapolation cm |
| max 2000 m | [E] | LOLA_16 (1.9 km) on the far side; the 20 m site tiles resolve ~100 m (generator fades by resolution) | - |
| d/D fresh 0.13 | [S] | Sun et al. 2018 median of ~850 fresh small craters (cited in arXiv:2008.10132); Stopar et al. 2017 (Icarus 298): 0.11-0.17 for fresh D < 400 m, ~0.12 for 30-100 m; Mahanti et al. 2018: freshest class 0.13-0.17 | 30-400 m measured |
| d/D degraded 0.07 | [E] | Mahanti et al. 2018 (Icarus 299): small craters (35-250 m) at Apollo 16/17 mostly degraded, mean d/D < 0.1, 99 % in 0.04-0.17; Minton et al. 2019: counting limit 0.05 | 35-250 m |
| rim height 0.024 D | [D] | Pike 1977: fresh simple craters d = 0.196 D, h_rim = 0.036 D -> rim/depth 0.184, applied to d/D 0.13 | km-scale ratio applied to small craters |
| degraded fraction 0.70 | [S] | Mahanti et al. 2018 via LROC news: ~70 % class C at Apollo 16 (highland), 88 % at Apollo 17 | 35-250 m |

Disagreement: equilibrium densities range 0.7 % (Xiao & Werner) to 5 % (Shoemaker & Morris' 10^10.9 if read as per
10^6 km^2 with D in m - units not confirmed from the original) of geometric saturation; 2.2 % (Minton, a modern
count close to the median of the Robbins et al. 2014 counter ensemble) lies in the middle.

### Regolith roughness
| Value | Tag | Source | Range |
|---|---|---|---|
| H = 0.78 | [S] | Guo et al. 2021 (GRL 48, e2021GL094931), Chang'e-4 5 mm DTM, sec. 3.2: H = 0.78, 0.85, 0.85, 0.71, 0.51 for the five natural subregions (0.61 rover tracks, excluded), median 0.78; breakover 0.5-2 m | 10 mm - 2.5 m |
| RMS 4 mm at 125 mm | [S] | Guo et al. 2021 sec. 3.1: RMS height (std. dev. in a window) median 4 mm at 125 mm, 1-18 mm | 15 mm - 1.25 m |

Disagreement: at metre-hectometre scales H ~ 0.9 (Cai 2019 LPSC 2343 / Cai & Fa 2020, NAC DTMs: maria and highlands
median 0.9; RMS height in 100 m windows 1.28 m maria, 3.90 m highlands); LOLA 17 m - 2.7 km: highlands 0.95, maria
0.76 (Rosenburg et al. 2011). Those include the craters, which we model explicitly, and lie above the 1-2 m
breakover; the cm-m regolith term follows Guo. Extrapolated to 100 m the term gives 0.74 m, below the NAC totals
with craters (consistent).

### Rocks
| Value | Tag | Source | Range |
|---|---|---|---|
| N(>D) = 0.154 D^-2.286 per 100 m^2 | [S] | Cintala & McBride 1995 (NASA TM-104804) Table 3: power-law fits to the Surveyor fragment counts of Shoemaker & Morris 1968; Surveyor VI (mare, mature regolith; exponent +-0.094) | mm - dm measured |
| other Surveyor sites | - | I: 0.797 D^-1.779; III: 0.049 D^-2.559; VII (Tycho ejecta, highland): 2.666 D^-1.802 | mm - dm |
| max 10 m | [E] | extrapolation | - |
| b/a 0.71, c/a 0.50 | [S] | Fujiwara et al. 1978 (Icarus 31) impact fragments a:b:c = 2:sqrt2:1; LRO NAC boulders b/a mostly ~0.7, 0.3-0.9 (Kumar et al., NGRI; Krishna & Kumar 2016) | NAC: m-dam |
| sd 0.15 | [E] | spread of the NAC range | - |
| burial 0.3 +- 0.15 | [E] | no lunar measurement found | - |
| albedo 1.35 x regolith | [E] | w_rock 0.65 / w_regolith 0.48 of the synthetic NAC boulder-field model of Marshal, Ruesch, Woehler et al. (EGU22-12217), taken there from earlier work; Ruesch et al. 2024 (JGR) found boulders near Reiner Gamma with w like the regolith | - |
| abundance Diviner | [S] | Bandfield et al. 2011 (JGR 116, doi:10.1029/2011JE003866), GDR L3 rock abundance, 60 S - 60 N, sensitive to rocks > ~0.1-1 m; typical < 1 % (Hayne et al. 2017) | regional |

Join of the ranges: Surveyor frames resolve mm-dm fragments; orbital counts (Lunar Orbiter, LRO NAC) start at ~1-2 m.
Cintala & McBride (1995) found the orbital (> m) slopes steeper (-3.5 to -6) than the Surveyor ones (~-2) and could
not decide whether one law spans both; Li et al. 2017 (LRO near the Surveyor sites, exponents -1.5 to -3.6) argue a
single power law fits mm to m. We extrapolate the Surveyor VI law and check it against the orbital data: it covers
0.68 % of the area with 0.5-10 m rocks (Diviner: typically < 1 %); Surveyor I/VII laws (slope ~1.8) would give
several % (Cintala & McBride: they overestimate the large blocks), Surveyor III is a crater interior. Steeper
decametre slopes (Bart & Melosh 2010: mostly -3 to -4.5 around fresh craters; Cintala & McBride Surveyor VII
orbital -4.0) belong to ejecta fields, which the abundance field raises locally.

### Hapke
| Value | Tag | Source |
|---|---|---|
| w 0.429, b 0.220, c 0.514, B_S0 1.61, h_S 0.054 | [D] | medians over the brighter half (w above the global median, highland proxy; 24,660 1-deg cells) of the PDS map WAC_HAPKEPARAMMAP_643NM (Sato et al. 2014, JGR Planets 119, 1775), computed by us; cells with fill values excluded. Global medians: 0.395, 0.228, 0.420, 1.66, 0.051; darker half (maria proxy): 0.353, 0.237, 0.327, 1.73, 0.049 |
| B_C0 0, h_C 1, theta_bar 23.657 deg, K 1 (filling factor 0) | [S] | fixed in Sato et al. 2014 (map README; the map's filling-factor band is 0, consistent with Barker et al. 2016 who state Sato used K = 1) |

Cross-check: Barker et al. 2016 (LOLA 1064 nm, Icarus) quote Sato's highlands b = 0.23-0.24, B_S0 1.60 at 689 nm,
h_S 0.075-0.08, maria h_S 0.05; our highland-proxy medians agree except h_S (0.054: the w-split mixes in
intermediate terrain). The map stops at 70 deg; 60-70 S cells give c 0.78, h_S 0.018 - not used, the polar opposition
geometry (g near 0) is not observed there. The published global-average table of Sato et al. was not accessible
(paywalled).

## Open points
- Diviner rock abundance does not cover the shipped site (-84.8 deg); there the uniform population applies. Download
  and the fallback are Task 3.
- Burial, rock albedo ratio and degraded d/D are estimates; no direct lunar measurements found.
- One Hurst exponent for cm to tens of m although the measured breakover is 1-2 m (H 0.78 below, ~0.9 above).
- Units of Shoemaker & Morris' 10^10.9 not confirmed from the original paper (used only as a comparison).
