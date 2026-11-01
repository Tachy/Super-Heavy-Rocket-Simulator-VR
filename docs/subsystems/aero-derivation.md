# Aero derivation (ascent and booster descent drag from the built models)

Cross-check of the implemented drag tables against the drawn geometry (plan `docs/superpowers/plans/2026-11-01-vehicle-physics.md`,
Tasks 1-2). Nothing in `config/` or `core/` depends on it; no number here was fitted; a disagreement is reported as it is. The user's
OK on these results decides Tasks 3 onward.

## Code and commands
- `tools/aero_derive/geometry.py`: built glTFs as triangles in body coordinates (ports `flap_transform`, `fin_transform`), secured
  areas. `python tools/aero_derive/geometry.py` writes `screenshots/vehicle-physics/secured_areas.csv`.
- `newtonian.py`: modified Newtonian with first-hit occlusion on a ray grid (winding unused, so lee sides and interiors never load).
- `buildup.py`: friction, nose, base, protuberance, cavity, grid fin; each returns (value, relative error) and cites its source.
- `derive_ascent_descent.py`: `python tools/aero_derive/derive_ascent_descent.py` writes to `screenshots/vehicle-physics/`:
  `drag_compare.csv`, `telemetry_bins.csv`, `tables.md`, `drag_ascent.png`, `drag_ascent_off.png`, `drag_descent_body.png`,
  `drag_descent.png`, `beta_descent.png`.
- Tests: `python -m pytest tools/aero_derive -q` (closed-form sphere, disc, cylinder, shadowing, flat plate; model areas; output rows).

## Secured areas
| Quantity | Drawn | Error | Config | Drawn / config | Source |
|---|---|---|---|---|---|
| flap aft area [m^2] | 36.480 | +-0.730 | 48 | 0.760 | detail outline, STL 1:100 [E]; mesh projection 36.48 |
| flap aft span [m] | 3.690 | +-0.037 | 4 | 0.922 | detail outline [E] |
| flap fwd area [m^2] | 12.374 | +-0.247 | 24 | 0.516 | detail outline, STL 1:100 [E]; mesh projection 12.37 |
| flap fwd span [m] | 3.154 | +-0.032 | 3 | 1.051 | detail outline [E] |
| grid fin gross area [m^2] | 12.626 | +-0.253 | 13.5 | 0.935 | detail outline, STL 1:100 [E]; 102.9 cells x 0.35^2 = 12.60 |
| grid fin open fraction [-] | 0.694 | +-0.040 | - | - | solid parts by first hit along the cells at d = 0, then the walls (1 - w/p)^2 = 0.784 (mesh walls are zero-thick sheets; wall 0.04 m is an upper bound [E]); band 0.06, frame 0.89, lattice 0.20, pylon 0.31 m^2 |
| grid fin open area [m^2] | 8.764 | +-0.505 | - | - | open fraction x gross |
| wetted hull booster [m^2] | 1965.453 | +-19.852 | - | - | lathe integral to station 69.42; mesh 1965.3; plain cylinder pi D L 2044.2 |
| wetted hull ship [m^2] | 1371.985 | +-13.899 | - | - | lathe integral to station 52.10; mesh 1371.8; plain cylinder pi D L 1473.1 |

The lattice walls are zero-thickness sheets in the mesh, so the ray test cannot see `wall_m`: the open fraction is solid parts by first
hit (frame, rib, pylon) plus walls from `wall_m` / pitch. The grid fin's `plate` is the renderer's far stand-in and is left out.

## Method
Reference area pi/4 D^2 = 63.617 m^2. Continuum only: the core blends to 2.2 above Kn 0.01, so where the flown altitude lies beyond
Kn 0.01 (Mach >= 8) friction is evaluated at the Kn 0.01 altitude and the row is marked `*` / "not flown in continuum".

- **Ascent:** flight 14's altitude at each Mach (dials). Friction (Sommer-Short + Prandtl-Schlichting) on 3337 m^2 of drawn hull; nose
  wave drag (Bonney in Fleeman) for the drawn 13.4 m ogive; base (Fleeman; power on: outside the 33 nozzle exits, 54.2 m^2); flaps
  **at d = 90, their rest pose in ascent (user fact 2026-11-01, flight 14 at T+49 s)**: thin radial fins, friction on both faces of the
  full 97.7 m^2 planform, edge and wedge pressure from the mesh; grid fins (below); all other drawn parts as protuberances with their
  mesh load; the hot-stage band as an open cavity. Above Mach 3 also the Newtonian mesh value of the nose.
- **Booster descent:** tail first, alone, fins at 0. Mach < 1.5: Hoerner's blunt face (0.76 of stagnation, rising with Mach), base and
  friction; the rim's separation bubble is taken to cover skirt and strake ends, so no protrusion term. Mach >= 1.5: Newtonian first
  hit of all drawn parts, base and friction (the plan's method, chosen before seeing results). Base on the closed hull disc; the bells'
  concave interiors stagnate in reality, so the pure Newtonian "with bells" column is a lower bound, not a candidate.
- **Grid fin** (per gross area): capture from one-dimensional choking of a throat of the open fraction; started: Idel'chik thick grid
  below Mach 1, Newtonian mesh + wall base above, plus cell friction; the spilled share acts as a solid plate.
- **Flights:** raw `drag_estimate.py` bins of flights 13 and 14 from T+340 s to relight (386 / 396 s), D/m >= 3 m/s^2, 261 t.
- **Errors:** derived, per term in `buildup.py`, added in quadrature. Table bands assumed here (none are published) [E]: ascent +-15 %,
  descent +-12.5 % where identified (Mach 1.1-3.9), +-20 % elsewhere. "Within": the difference is inside the combined error.

## Results
### Ascent, stack

| Mach | h km | col | friction | nose | base | clean (sum) | flaps | fins | other | gap | derived | +- | table | derived/table | verdict | Newtonian | nose 17 m | flaps d=0 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0.3 | 0.9 | on | 0.084 | 0.000 | 0.020 | 0.104 | 0.039 | 0.330 | 0.129 | 0.053 | 0.655 | 0.126 | 0.300 | 2.18 | outside | - | 0.655 | 0.624 |
| 0.3 | 0.9 | off | 0.084 | 0.000 | 0.132 | 0.216 | 0.039 | 0.330 | 0.129 | 0.053 | 0.767 | 0.132 | 0.380 | 2.02 | outside | - | 0.767 | 0.736 |
| 0.6 | 3.0 | on | 0.077 | 0.000 | 0.025 | 0.102 | 0.042 | 0.374 | 0.141 | 0.053 | 0.712 | 0.204 | 0.300 | 2.37 | outside | - | 0.712 | 0.678 |
| 0.6 | 3.0 | off | 0.077 | 0.000 | 0.167 | 0.244 | 0.042 | 0.374 | 0.141 | 0.053 | 0.854 | 0.210 | 0.380 | 2.25 | outside | - | 0.854 | 0.820 |
| 0.8 | 4.4 | on | 0.074 | 0.000 | 0.030 | 0.104 | 0.045 | 0.417 | 0.155 | 0.053 | 0.774 | 0.226 | 0.330 | 2.35 | outside | - | 0.774 | 0.737 |
| 0.8 | 4.4 | off | 0.074 | 0.000 | 0.203 | 0.277 | 0.045 | 0.417 | 0.155 | 0.053 | 0.947 | 0.233 | 0.410 | 2.31 | outside | - | 0.947 | 0.910 |
| 0.95 | 5.6 | on | 0.072 | 0.215 | 0.035 | 0.322 | 0.048 | 0.441 | 0.168 | 0.053 | 1.032 | 0.262 | 0.450 | 2.29 | outside | - | 0.964 | 0.992 |
| 0.95 | 5.6 | off | 0.072 | 0.215 | 0.237 | 0.524 | 0.048 | 0.441 | 0.168 | 0.053 | 1.234 | 0.270 | 0.550 | 2.24 | outside | - | 1.166 | 1.194 |
| 1.05 | 6.4 | on | 0.071 | 0.484 | 0.035 | 0.590 | 0.049 | 0.418 | 0.175 | 0.053 | 1.285 | 0.333 | 0.580 | 2.22 | outside | - | 1.132 | 1.243 |
| 1.05 | 6.4 | off | 0.071 | 0.484 | 0.238 | 0.792 | 0.049 | 0.418 | 0.175 | 0.053 | 1.488 | 0.340 | 0.680 | 2.19 | outside | - | 1.335 | 1.446 |
| 1.2 | 7.9 | on | 0.069 | 0.426 | 0.031 | 0.526 | 0.051 | 0.427 | 0.183 | 0.053 | 1.240 | 0.257 | 0.620 | 2.00 | outside | - | 1.105 | 1.196 |
| 1.2 | 7.9 | off | 0.069 | 0.426 | 0.208 | 0.703 | 0.051 | 0.427 | 0.183 | 0.053 | 1.417 | 0.264 | 0.720 | 1.97 | outside | - | 1.283 | 1.373 |
| 1.5 | 10.7 | on | 0.066 | 0.357 | 0.025 | 0.448 | 0.054 | 0.429 | 0.196 | 0.053 | 1.181 | 0.253 | 0.570 | 2.07 | outside | - | 1.068 | 1.134 |
| 1.5 | 10.7 | off | 0.066 | 0.357 | 0.167 | 0.590 | 0.054 | 0.429 | 0.196 | 0.053 | 1.323 | 0.257 | 0.660 | 2.00 | outside | - | 1.210 | 1.276 |
| 2 | 15.1 | on | 0.062 | 0.304 | 0.019 | 0.384 | 0.058 | 0.407 | 0.213 | 0.053 | 1.116 | 0.241 | 0.500 | 2.23 | outside | - | 1.019 | 1.064 |
| 2 | 15.1 | off | 0.062 | 0.304 | 0.125 | 0.491 | 0.058 | 0.407 | 0.213 | 0.053 | 1.222 | 0.244 | 0.580 | 2.11 | outside | - | 1.126 | 1.171 |
| 3 | 24.0 | on | 0.055 | 0.266 | 0.012 | 0.333 | 0.063 | 0.344 | 0.236 | 0.053 | 1.030 | 0.216 | 0.400 | 2.58 | outside | 0.972 | 0.946 | 0.974 |
| 3 | 24.0 | off | 0.055 | 0.266 | 0.083 | 0.404 | 0.063 | 0.344 | 0.236 | 0.053 | 1.101 | 0.217 | 0.470 | 2.34 | outside | 1.043 | 1.017 | 1.045 |
| 5 | 51.2 | on | 0.059 | 0.247 | 0.007 | 0.314 | 0.063 | 0.338 | 0.238 | 0.053 | 1.006 | 0.164 | 0.325 | 3.10 | outside | 0.973 | 0.928 | 0.949 |
| 5 | 51.2 | off | 0.059 | 0.247 | 0.050 | 0.356 | 0.063 | 0.338 | 0.238 | 0.053 | 1.049 | 0.164 | 0.380 | 2.76 | outside | 1.016 | 0.971 | 0.991 |
| 8 | 113.4* | on | 1.450 | 0.240 | 0.003 | 1.693 | 0.363 | 0.859 | 0.236 | 0.053 | 3.204 | 0.602 | 0.285 | 11.24 | outside (not flown in continuum) | 3.181 | 3.128 | 3.147 |
| 8 | 113.4* | off | 1.450 | 0.240 | 0.022 | 1.712 | 0.363 | 0.859 | 0.236 | 0.053 | 3.223 | 0.602 | 0.330 | 9.77 | outside (not flown in continuum) | 3.200 | 3.147 | 3.166 |
| 15 | 113.4* | on | 0.809 | 0.237 | 0.001 | 1.047 | 0.210 | 0.485 | 0.235 | 0.053 | 2.031 | 0.352 | 0.263 | 7.71 | outside (not flown in continuum) | 2.012 | 1.956 | 1.975 |
| 15 | 113.4* | off | 0.809 | 0.237 | 0.006 | 1.053 | 0.210 | 0.485 | 0.235 | 0.053 | 2.037 | 0.352 | 0.303 | 6.71 | outside (not flown in continuum) | 2.017 | 1.962 | 1.980 |
| 25 | 113.4* | on | 0.492 | 0.237 | 0.000 | 0.729 | 0.114 | 0.379 | 0.235 | 0.053 | 1.510 | 0.246 | 0.250 | 6.04 | outside (not flown in continuum) | 1.492 | 1.435 | 1.454 |
| 25 | 113.4* | off | 0.492 | 0.237 | 0.002 | 0.731 | 0.114 | 0.379 | 0.235 | 0.053 | 1.512 | 0.246 | 0.290 | 5.21 | outside (not flown in continuum) | 1.494 | 1.437 | 1.456 |

### Ascent, ship alone

| Mach | h km | col | friction | nose | base | clean (sum) | flaps | fins | other | gap | derived | +- | table | derived/table | verdict | Newtonian | nose 17 m | flaps d=0 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 5 | 51.2 | on | 0.011 | 0.247 | 0.035 | 0.293 | 0.062 | 0.000 | 0.056 | 0.000 | 0.411 | 0.073 | 0.325 | 1.27 | within | 0.379 | 0.333 | - |
| 5 | 51.2 | off | 0.011 | 0.247 | 0.050 | 0.308 | 0.062 | 0.000 | 0.056 | 0.000 | 0.426 | 0.071 | 0.380 | 1.12 | within | 0.393 | 0.348 | - |
| 8 | 108.4* | on | 0.596 | 0.240 | 0.016 | 0.852 | 0.255 | 0.000 | 0.055 | 0.000 | 1.162 | 0.232 | 0.285 | 4.08 | outside (not flown in continuum) | 1.139 | 1.086 | - |
| 8 | 108.4* | off | 0.596 | 0.240 | 0.022 | 0.859 | 0.255 | 0.000 | 0.055 | 0.000 | 1.169 | 0.232 | 0.330 | 3.54 | outside (not flown in continuum) | 1.145 | 1.093 | - |
| 15 | 108.4* | on | 0.333 | 0.237 | 0.005 | 0.574 | 0.168 | 0.000 | 0.055 | 0.000 | 0.797 | 0.141 | 0.263 | 3.03 | outside (not flown in continuum) | 0.778 | 0.722 | - |
| 15 | 108.4* | off | 0.333 | 0.237 | 0.006 | 0.576 | 0.168 | 0.000 | 0.055 | 0.000 | 0.799 | 0.141 | 0.303 | 2.63 | outside (not flown in continuum) | 0.780 | 0.724 | - |
| 25 | 108.4* | on | 0.202 | 0.237 | 0.002 | 0.440 | 0.114 | 0.000 | 0.055 | 0.000 | 0.609 | 0.100 | 0.250 | 2.44 | outside (not flown in continuum) | 0.591 | 0.534 | - |
| 25 | 108.4* | off | 0.202 | 0.237 | 0.002 | 0.441 | 0.114 | 0.000 | 0.055 | 0.000 | 0.610 | 0.100 | 0.290 | 2.10 | outside (not flown in continuum) | 0.591 | 0.535 | - |

### Booster descent, body without fins

| Mach | band | face | friction | base | derived | +- | table | derived/table | verdict | hull face only | Hoerner face | with bells |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0.3 | subsonic | 0.777 | 0.053 | 0.132 | 0.962 | 0.088 | 0.925 | 1.04 | within | 1.149 | 0.962 | 0.798 |
| 0.6 | subsonic | 0.831 | 0.048 | 0.167 | 1.045 | 0.097 | 0.950 | 1.10 | within | 1.245 | 1.045 | 0.869 |
| 0.8 | transonic | 0.890 | 0.045 | 0.203 | 1.138 | 0.188 | 1.000 | 1.14 | within | 1.352 | 1.138 | 0.949 |
| 0.95 | transonic | 0.947 | 0.043 | 0.237 | 1.228 | 0.203 | 1.150 | 1.07 | within | 1.456 | 1.228 | 1.027 |
| 1.05 | transonic | 0.997 | 0.043 | 0.238 | 1.278 | 0.212 | 1.350 | 0.95 | within | 1.513 | 1.278 | 1.063 |
| 1.2 | transonic | 1.081 | 0.041 | 0.208 | 1.331 | 0.225 | 1.432 | 0.93 | within | 1.566 | 1.331 | 1.086 |
| 1.5 | supersonic | 1.766 | 0.039 | 0.167 | 1.971 | 0.357 | 1.380 | 1.43 | outside | 1.650 | 1.424 | 1.123 |
| 2 | supersonic | 1.910 | 0.034 | 0.125 | 2.069 | 0.384 | 1.420 | 1.46 | outside | 1.722 | 1.535 | 1.152 |
| 3 | supersonic | 2.023 | 0.033 | 0.083 | 2.140 | 0.405 | 1.750 | 1.22 | within | 1.772 | 1.697 | 1.168 |
| 5 | supersonic | 2.084 | 0.021 | 0.050 | 2.155 | 0.313 | 1.880 | 1.15 | within | 1.777 | 1.699 | 1.154 |
| 8 | supersonic | 2.106 | 0.012 | 0.022 | 2.140 | 0.316 | 1.880 | 1.14 | within | 1.757 | 1.679 | 1.129 |
| 15 | supersonic | 2.115 | 0.005 | 0.006 | 2.126 | 0.317 | 1.880 | 1.13 | within | 1.742 | 1.663 | 1.110 |
| 25 | supersonic | 2.118 | 0.002 | 0.002 | 2.122 | 0.318 | 1.880 | 1.13 | within | 1.737 | 1.659 | 1.105 |

### Booster descent, body + fins

| Mach | fin CA derived (gross) | +- | fin CA table | total, 12.63 | total, 13.5 | +- | table total | verdict (12.63 / 13.5) | flights C_D at 261 t (sd, n) | verdict against flights (12.63 / 13.5) | beta 12.63 | beta 13.5 | beta tables | beta flights (sd) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0.3 | 0.554 | 30 % | 0.821 | 1.292 | 1.315 | 0.132 | 1.448 | within / within | - | no flight data / no flight data | 3174 | 3119 | 2833 | - |
| 0.6 | 0.628 | 50 % | 0.843 | 1.419 | 1.445 | 0.211 | 1.487 | within / within | - | no flight data / no flight data | 2891 | 2839 | 2760 | - |
| 0.8 | 0.699 | 50 % | 0.900 | 1.554 | 1.583 | 0.281 | 1.573 | within / within | - | no flight data / no flight data | 2640 | 2592 | 2608 | - |
| 0.95 | 0.740 | 50 % | 0.962 | 1.668 | 1.699 | 0.299 | 1.763 | within / within | - | no flight data / no flight data | 2460 | 2415 | 2327 | - |
| 1.05 | 0.554 | 50 % | 0.987 | 1.608 | 1.631 | 0.269 | 1.979 | within / within | 2.06 (-, 1) | outside / outside | 2551 | 2515 | 2073 | 1995 (-) |
| 1.2 | 0.557 | 50 % | 1.000 | 1.662 | 1.685 | 0.280 | 2.069 | outside / outside | 2.28 (0.14, 4) | outside / outside | 2469 | 2435 | 1983 | 1804 (108) |
| 1.5 | 0.532 | 50 % | 0.983 | 2.288 | 2.310 | 0.390 | 2.006 | within / within | 1.71 (0.17, 3) | outside / outside | 1793 | 1776 | 2045 | 2410 (232) |
| 2 | 0.456 | 50 % | 0.900 | 2.340 | 2.359 | 0.407 | 1.993 | within / within | 1.91 (0.18, 6) | within / outside | 1753 | 1739 | 2059 | 2162 (201) |
| 3 | 0.313 | 50 % | 0.800 | 2.326 | 2.339 | 0.416 | 2.259 | within / within | 2.28 (0.18, 23) | within / within | 1764 | 1754 | 1816 | 1809 (141) |
| 5 | 0.251 | 30 % | 0.750 | 2.305 | 2.316 | 0.316 | 2.357 | within / within | 2.34 (-, 1) | within / within | 1780 | 1772 | 1740 | 1751 (-) |
| 8 | 0.245 | 30 % | 0.750 | 2.286 | 2.296 | 0.319 | 2.357 | within / within | - | no flight data / no flight data | 1795 | 1787 | 1740 | - |
| 15 | 0.239 | 30 % | 0.750 | 2.269 | 2.278 | 0.320 | 2.357 | within / within | - | no flight data / no flight data | 1808 | 1801 | 1740 | - |
| 25 | 0.237 | 30 % | 0.750 | 2.263 | 2.273 | 0.321 | 2.357 | within / within | - | no flight data / no flight data | 1813 | 1805 | 1740 | - |

### Mesh factors k (per unit cp_max, on 63.6 m^2)

- Tail first, no bells: booster/hull 0.9428, booster/skirt 0.1657, booster/strakes 0.0437, fin 0.0396
- Tail first, drawn bells: booster/hull 0.2645, engine 0.2207, booster/skirt 0.0700, booster/strakes 0.0437, fin 0.0396
- Stack nose first, flaps 90: ship/hull 0.1186, booster/skirt 0.0635, fin 0.0587, booster/top_truss 0.0423, flap 0.0353, ship/hinge_covers 0.0130, ship/catch_pins 0.0121, ship/ports 0.0065, booster/strakes 0.0025, ship/raceway 0.0009, ship/chine 0.0009, booster/raceway 0.0000, booster/fin_fairings 0.0000
- Stack nose first, flaps 0: ship/hull 0.1152, fin 0.0592, booster/skirt 0.0448, booster/top_truss 0.0327, flap 0.0299, ship/hinge_covers 0.0135, ship/catch_pins 0.0121, ship/ports 0.0065, booster/strakes 0.0025, ship/raceway 0.0009, booster/raceway 0.0000, booster/fin_fairings 0.0000, ship/chine 0.0000
- Ship nose first, flaps 90: ship/hull 0.1186, flap 0.0350, ship/hinge_covers 0.0130, ship/catch_pins 0.0119, ship/ports 0.0064, ship/raceway 0.0010, ship/chine 0.0008
- Fin, per gross area: tail first 0.128, nose first 0.289; aft wall edge mean cos^2 0.322
- Open fraction 0.694
- Flight 14, stack to T+160 s: peak q 38.9 kPa at T+63 s (Mach 1.36, 9.2 km). Drag / thrust there: derived 3.3 %, table 1.6 %; their difference is 0.35 m/s^2 of acceleration. Velocity lost to drag: derived 39 m/s, table 17 m/s.

Plots: `screenshots/vehicle-physics/*.png`.

## Verdicts per Mach band

| Case | Mach | Verdict | Reason, as far as known |
|---|---|---|---|
| Ascent stack, both columns | 0.3-5 | **outside**, derived / table 2.0-3.1 | Fins 0.33-0.44 and other drawn parts 0.13-0.24 are the excess. The table is a generic slender-vehicle trend without fins. Without fins and other parts (clean + flaps + gap), power on, Mach 1.05-5: 0.43-0.69 against 0.32-0.62; ratio 0.97-1.32 |
| Ascent stack | 8-25 | outside, not a flown state | The stack stages at about Mach 6. Friction at Kn 0.01 is laminar and at the end of its range |
| Ascent ship alone | 5 | within (1.27 on / 1.12 off) | The table's use after separation holds here |
| Ascent ship alone | 8-25 | outside, ratio 2.1-4.1, not flown in continuum | Friction near Kn 0.01; at the flown altitude the core's free-molecular blend dominates and the table's weight is small |
| Descent body | 0.3-1.2 | within (0.93-1.14) | Hoerner blunt body; subsonic is unmeasured on both sides |
| Descent body | 1.5, 2 | **outside** (1.43, 1.46) | Newtonian loads the flat face with full stagnation pressure, and the protrusions (k 0.21). Hoerner's face gives 1.42 / 1.54 against 1.38 / 1.42, so the method jump at Mach 1.5 is most of it |
| Descent body | 3-25 | within (1.13-1.22) | Hull face alone: 1.77 at Mach 3-5 against 1.75-1.88 |
| Descent total | all but 1.2 | within the table | Body high and fins low compensate from Mach 1.5 up: derived fin CA 0.24-0.53 against the table's 0.75-0.98. The agreement of the sum does not confirm the split |
| Descent total | 1.2 | outside the table (1.66 against 2.07) | Fin form changes at Mach 1 and the blunt-face trend is the least certain band |
| Descent total against flights | 1.05, 1.2 | **outside**: 1.61, 1.66 against 2.06 (one bin), 2.28 | Derived too low in the band flagged least certain |
| Descent total against flights | 1.5 | **outside**: 2.29 against 1.71 | As descent body at 1.5 |
| Descent total against flights | 2 | within at 12.63 m^2, outside at 13.5 m^2 | A thin margin, not a finding about the area |
| Descent total against flights | 3, 5 | within: 2.33, 2.31 against 2.28, 2.34 (one bin) | Beta 1764 / 1780 against 1809 / 1751 kg/m^2; Mach 3 is the densest band (23 bins) |

What the flights say about the ascent: **they cannot resolve its Cd.** At peak q (38.9 kPa, T+63 s) the derived and table drag differ by
0.35 m/s^2, 1.7 % of thrust, less than the thrust change of one engine out (flight 14 flew 32 of 33). Velocity lost to drag up to T+160 s
is 39 m/s derived and 17 m/s table, with the mass history assumed. `late_ascent_fit.py` works where drag is under 0.5 % of thrust and
does not separate it.

## Sensitivities
- Fin area 13.5 against 12.63 m^2: descent total +0.010 to +0.030 (at most 1.9 %), beta -8 to -55 kg/m^2.
- Flaps at d = 0 instead of 90 in ascent (note only, d = 90 is the real pose): stack -0.031 to -0.057 (includes what folded flaps shadow).
  Flaps at d = 90 carry 0.039-0.063 in the stack at Mach 0.3-5.
- Nose 17 m (physics) instead of 13.4 m (drawn): -0.07 to -0.15 at Mach 0.95-5, nothing below.
- Drawn bells in the pure Newtonian descent: body 1.12-1.17 at Mach >= 1.5, a lower bound (see Method).
- Hull face only against all drawn parts, descent Mach >= 1.5: 1.65-1.78 against 1.97-2.16.

## Limits
- Book constants (Hoerner, Fleeman, Idel'chik) were recalled, not re-read from a printed copy; the bands are wide for this and marked [E].
- No boundary-layer credit for protrusions (skirt rings stand partly in the layer at the stack's aft end), which makes the ascent
  "other" term an upper estimate. Rods, strakes and skirt have no wind-tunnel reference here.
- No interference between fins and hull, no real-gas cp_max above Mach ~8. The fin's started form jumps at Mach 1 (Idel'chik to Newtonian).
- Grid fin wall thickness 0.04 m is an upper bound (not resolved in any frame); it sets the open fraction 0.694 +-0.04.
- Stack rows at Mach >= 8 and descent rows at Mach >= 5 are not measured states: descent friction there uses a nominal 30 km.

## Open
The user decides on this comparison before Tasks 3-7 (no table is re-fitted, user 2026-10-31).
