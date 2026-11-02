# Aero derivation (ascent and booster descent drag from the built models)

Drag derived from the drawn geometry (plan `docs/superpowers/archive/2026-11-01-vehicle-physics.md`, Tasks 1, 2, 2b, 4, 4b, 4c). The stack's ascent
table and the Super Heavy's fin table are this tool's output; the descent tables and the ship's table are only compared. No number here was fitted; a disagreement is
reported as it is.

## Code and commands
- `tools/aero_derive/geometry.py`: built glTFs as triangles in body coordinates (ports `flap_transform`, `fin_transform`), secured
  areas. `python tools/aero_derive/geometry.py` writes `screenshots/vehicle-physics/secured_areas.csv`.
- `newtonian.py`: modified Newtonian with first-hit occlusion on a ray grid (winding unused, so lee sides and interiors never load).
- `exposure.py`: nose-first load with a wake range: which samples see the flow, with their station and height for the layer credit.
- `buildup.py`: friction, nose, base, protuberance, cavity, grid fin; each returns (value, relative error) and cites its source.
- `derive_ascent_descent.py`: `python tools/aero_derive/derive_ascent_descent.py` writes to `screenshots/vehicle-physics/`:
  `drag_compare.csv`, `telemetry_bins.csv`, `tables.md`, `drag_ascent.png`, `drag_ascent_off.png`, `drag_descent_body.png`,
  `drag_descent.png`, `beta_descent.png`.
- Tests: `python -m pytest tools/aero_derive -q` (closed-form sphere, disc, cylinder, shadowing, flat plate; model areas; output rows).

## Secured areas
| Quantity | Drawn | Error | Config | Drawn / config | Source |
|---|---|---|---|---|---|
| flap aft area [m^2] | 36.480 | +-0.730 | 36.48 | 1.000 | detail outline, STL 1:100 [E]; mesh projection 36.48 |
| flap aft span [m] | 3.690 | +-0.037 | 3.69 | 1.000 | detail outline [E] |
| flap fwd area [m^2] | 12.374 | +-0.247 | 12.374 | 1.000 | detail outline, STL 1:100 [E]; mesh projection 12.37 |
| flap fwd span [m] | 3.154 | +-0.032 | 3.154 | 1.000 | detail outline [E] |
| grid fin gross area [m^2] | 12.626 | +-0.253 | 12.63 | 1.000 | detail outline, STL 1:100 [E]; 102.9 cells x 0.35^2 = 12.60 |
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
  full 97.7 m^2 planform, edge and wedge pressure from the mesh; grid fins nose first (below), the count read from config; all other
  drawn parts as protuberances with their mesh load and layer credit; the hot-stage band as one term. Above Mach 3 also the Newtonian
  mesh value of the nose.
- **Exposure, nose first (`exposure.py`):** along each ray from the nose: the first hit loads; the hull ends the ray; a part already met
  does not load again; another part loads if it lies more than 10 of the passed part's heights behind that part ([E], turbulent step
  reattachment 6-8 heights, recalled; rounded up). The plain first hit let a ring 70 m upstream hide the skirt (k 0.0635 against 0.139).
  Descent stays on the plain first hit: its tables are not derived here (at most about 5 % of fin CA at Mach 5, Mach >= 1.5).
- **Boundary layer credit:** each exposed sample of the parts term counts at its local q / q_e = (y / delta)^(2/7) (1/7 power profile,
  1 above the layer), y = height out of the bare hull at that station, delta = 0.37 x Re_x^-0.2 with x from the nose. Over a band from
  the wall to h this is (7/9)(h / delta)^(2/7). Not applied to fins and flaps: they stand several layer thicknesses out (at most 3 % at
  peak q) and carry their own methods.
- **Hot-stage band, one term:** the cavity on the open height 2.56 m (barrel end at station 69.42 to the closed 0.32 m ring under the ship,
  `dome_start_m` less the ring) gives 0.034, +-100 %. The truss and the flange stand in that band and are not in the parts term. The
  truss as the band's term instead (k 0.048 x credit 0.65): 0.040 at Mach 1.2, 0.052 at Mach 5, inside the cavity's error.
- **Booster descent:** tail first, alone, fins at 0. Mach < 1.5: Hoerner's blunt face (0.76 of stagnation, rising with Mach), base and
  friction; the rim's separation bubble is taken to cover skirt and strake ends, so no protrusion term. Mach >= 1.5: Newtonian first
  hit of all drawn parts, base and friction (the plan's method, chosen before seeing results). Base on the closed hull disc; the bells'
  concave interiors stagnate in reality, so the pure Newtonian "with bells" column is a lower bound, not a candidate.
- **Grid fin** (per gross area, direction-aware): capture from one-dimensional choking of a throat of the open fraction; started:
  Idel'chik thick grid below Mach 1, with tau from the mean cell depth of the face met first (flat forward face 0.50 m: tau 0.063;
  scooped aft face 0.32 m: tau 0.22); Newtonian mesh of that direction + wall base above, plus cell friction; the spilled share acts as a
  solid plate. Nose first (flat forward face, k 0.296) for the stack, tail first (scooped aft face, k 0.128) for the descent.
- **Flights:** raw `drag_estimate.py` bins of flights 13 and 14 from T+340 s to relight (386 / 396 s), D/m >= 3 m/s^2, 261 t.
- **Errors:** derived, per term in `buildup.py`, added in quadrature. Table bands assumed here (none are published) [E]: ascent +-15 %,
  descent +-12.5 % where identified (Mach 1.1-3.9), +-20 % elsewhere. "Within": the difference is inside the combined error.

## Results
### Ascent, stack

| Mach | h km | col | friction | nose | base | clean (sum) | flaps | fins | other | other, no credit | gap | derived | +- | old table | derived/old | verdict | Newtonian | nose 17 m | flaps d=0 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0.3 | 0.9 | on | 0.084 | 0.000 | 0.020 | 0.104 | 0.045 | 0.127 | 0.115 | 0.168 | 0.034 | 0.424 | 0.073 | 0.300 | 1.41 | outside | - | 0.424 | 0.418 |
| 0.3 | 0.9 | off | 0.084 | 0.000 | 0.132 | 0.216 | 0.045 | 0.127 | 0.115 | 0.168 | 0.034 | 0.536 | 0.082 | 0.380 | 1.41 | outside | - | 0.536 | 0.530 |
| 0.6 | 3.0 | on | 0.077 | 0.000 | 0.025 | 0.102 | 0.048 | 0.134 | 0.129 | 0.184 | 0.034 | 0.447 | 0.095 | 0.300 | 1.49 | outside | - | 0.447 | 0.440 |
| 0.6 | 3.0 | off | 0.077 | 0.000 | 0.167 | 0.244 | 0.048 | 0.134 | 0.129 | 0.184 | 0.034 | 0.589 | 0.106 | 0.380 | 1.55 | outside | - | 0.589 | 0.582 |
| 0.8 | 4.4 | on | 0.074 | 0.000 | 0.030 | 0.104 | 0.052 | 0.199 | 0.142 | 0.202 | 0.034 | 0.531 | 0.123 | 0.330 | 1.61 | outside | - | 0.531 | 0.524 |
| 0.8 | 4.4 | off | 0.074 | 0.000 | 0.203 | 0.277 | 0.052 | 0.199 | 0.142 | 0.202 | 0.034 | 0.704 | 0.136 | 0.410 | 1.72 | outside | - | 0.704 | 0.697 |
| 0.95 | 5.6 | on | 0.072 | 0.215 | 0.035 | 0.322 | 0.056 | 0.224 | 0.154 | 0.219 | 0.034 | 0.790 | 0.174 | 0.450 | 1.76 | outside | - | 0.722 | 0.782 |
| 0.95 | 5.6 | off | 0.072 | 0.215 | 0.237 | 0.524 | 0.056 | 0.224 | 0.154 | 0.219 | 0.034 | 0.992 | 0.186 | 0.550 | 1.80 | outside | - | 0.924 | 0.984 |
| 1.05 | 6.4 | on | 0.071 | 0.484 | 0.035 | 0.590 | 0.058 | 0.262 | 0.161 | 0.228 | 0.034 | 1.104 | 0.286 | 0.580 | 1.90 | outside | - | 0.951 | 1.096 |
| 1.05 | 6.4 | off | 0.071 | 0.484 | 0.238 | 0.792 | 0.058 | 0.262 | 0.161 | 0.228 | 0.034 | 1.307 | 0.295 | 0.680 | 1.92 | outside | - | 1.154 | 1.299 |
| 1.2 | 7.9 | on | 0.069 | 0.426 | 0.031 | 0.526 | 0.060 | 0.262 | 0.168 | 0.238 | 0.034 | 1.049 | 0.187 | 0.620 | 1.69 | outside | - | 0.914 | 1.040 |
| 1.2 | 7.9 | off | 0.069 | 0.426 | 0.208 | 0.703 | 0.060 | 0.262 | 0.168 | 0.238 | 0.034 | 1.226 | 0.197 | 0.720 | 1.70 | outside | - | 1.092 | 1.217 |
| 1.5 | 10.7 | on | 0.066 | 0.357 | 0.025 | 0.448 | 0.063 | 0.240 | 0.179 | 0.255 | 0.034 | 0.965 | 0.172 | 0.570 | 1.69 | outside | - | 0.852 | 0.956 |
| 1.5 | 10.7 | off | 0.066 | 0.357 | 0.167 | 0.590 | 0.063 | 0.240 | 0.179 | 0.255 | 0.034 | 1.107 | 0.179 | 0.660 | 1.68 | outside | - | 0.994 | 1.098 |
| 2 | 15.1 | on | 0.062 | 0.304 | 0.019 | 0.384 | 0.068 | 0.185 | 0.191 | 0.277 | 0.034 | 0.862 | 0.130 | 0.500 | 1.72 | outside | - | 0.766 | 0.852 |
| 2 | 15.1 | off | 0.062 | 0.304 | 0.125 | 0.491 | 0.068 | 0.185 | 0.191 | 0.277 | 0.034 | 0.969 | 0.134 | 0.580 | 1.67 | outside | - | 0.873 | 0.959 |
| 3 | 24.0 | on | 0.055 | 0.266 | 0.012 | 0.333 | 0.074 | 0.184 | 0.202 | 0.307 | 0.034 | 0.829 | 0.127 | 0.400 | 2.07 | outside | 0.771 | 0.745 | 0.817 |
| 3 | 24.0 | off | 0.055 | 0.266 | 0.083 | 0.404 | 0.074 | 0.184 | 0.202 | 0.307 | 0.034 | 0.900 | 0.129 | 0.470 | 1.91 | outside | 0.842 | 0.816 | 0.888 |
| 4 | 33.9 | on | 0.051 | 0.253 | 0.009 | 0.314 | 0.075 | 0.188 | 0.193 | 0.309 | 0.034 | 0.804 | 0.124 | 0.350 | 2.30 | outside | 0.763 | 0.724 | 0.792 |
| 4 | 33.9 | off | 0.051 | 0.253 | 0.062 | 0.367 | 0.075 | 0.188 | 0.193 | 0.309 | 0.034 | 0.857 | 0.125 | 0.410 | 2.09 | outside | 0.816 | 0.777 | 0.845 |
| 5 | 51.2 | on | 0.059 | 0.247 | 0.007 | 0.314 | 0.074 | 0.200 | 0.174 | 0.310 | 0.034 | 0.797 | 0.120 | 0.325 | 2.45 | outside | 0.764 | 0.719 | 0.785 |
| 5 | 51.2 | off | 0.059 | 0.247 | 0.050 | 0.356 | 0.074 | 0.200 | 0.174 | 0.310 | 0.034 | 0.839 | 0.121 | 0.380 | 2.21 | outside | 0.807 | 0.761 | 0.828 |
| 8 | 113.4* | on | 1.450 | 0.240 | 0.003 | 1.693 | 0.374 | 0.722 | 0.107 | 0.308 | 0.034 | 2.930 | 0.578 | 0.285 | 10.28 | outside (not flown in continuum) | 2.906 | 2.854 | 2.919 |
| 8 | 113.4* | off | 1.450 | 0.240 | 0.022 | 1.712 | 0.374 | 0.722 | 0.107 | 0.308 | 0.034 | 2.949 | 0.578 | 0.330 | 8.94 | outside (not flown in continuum) | 2.926 | 2.873 | 2.938 |
| 15 | 113.4* | on | 0.809 | 0.237 | 0.001 | 1.047 | 0.221 | 0.349 | 0.111 | 0.306 | 0.034 | 1.762 | 0.325 | 0.263 | 6.69 | outside (not flown in continuum) | 1.743 | 1.687 | 1.751 |
| 15 | 113.4* | off | 0.809 | 0.237 | 0.006 | 1.053 | 0.221 | 0.349 | 0.111 | 0.306 | 0.034 | 1.768 | 0.325 | 0.303 | 5.83 | outside (not flown in continuum) | 1.748 | 1.693 | 1.756 |
| 25 | 113.4* | on | 0.492 | 0.237 | 0.000 | 0.729 | 0.125 | 0.243 | 0.114 | 0.306 | 0.034 | 1.245 | 0.211 | 0.250 | 4.98 | outside (not flown in continuum) | 1.226 | 1.170 | 1.233 |
| 25 | 113.4* | off | 0.492 | 0.237 | 0.002 | 0.731 | 0.125 | 0.243 | 0.114 | 0.306 | 0.034 | 1.247 | 0.211 | 0.290 | 4.30 | outside (not flown in continuum) | 1.228 | 1.172 | 1.235 |

### Ascent, ship alone

| Mach | h km | col | friction | nose | base | clean (sum) | flaps | fins | other | other, no credit | gap | derived | +- | old table | derived/old | verdict | Newtonian | nose 17 m | flaps d=0 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 5 | 51.2 | on | 0.011 | 0.247 | 0.035 | 0.293 | 0.074 | 0.000 | 0.051 | 0.058 | 0.000 | 0.419 | 0.074 | 0.325 | 1.29 | outside | 0.386 | 0.341 | - |
| 5 | 51.2 | off | 0.011 | 0.247 | 0.050 | 0.308 | 0.074 | 0.000 | 0.051 | 0.058 | 0.000 | 0.433 | 0.073 | 0.380 | 1.14 | within | 0.401 | 0.355 | - |
| 8 | 108.4* | on | 0.596 | 0.240 | 0.016 | 0.852 | 0.267 | 0.000 | 0.036 | 0.058 | 0.000 | 1.155 | 0.232 | 0.285 | 4.05 | outside (not flown in continuum) | 1.131 | 1.079 | - |
| 8 | 108.4* | off | 0.596 | 0.240 | 0.022 | 0.859 | 0.267 | 0.000 | 0.036 | 0.058 | 0.000 | 1.162 | 0.232 | 0.330 | 3.52 | outside (not flown in continuum) | 1.138 | 1.086 | - |
| 15 | 108.4* | on | 0.333 | 0.237 | 0.005 | 0.574 | 0.179 | 0.000 | 0.037 | 0.057 | 0.000 | 0.791 | 0.141 | 0.263 | 3.01 | outside (not flown in continuum) | 0.772 | 0.716 | - |
| 15 | 108.4* | off | 0.333 | 0.237 | 0.006 | 0.576 | 0.179 | 0.000 | 0.037 | 0.057 | 0.000 | 0.793 | 0.141 | 0.303 | 2.61 | outside (not flown in continuum) | 0.774 | 0.718 | - |
| 25 | 108.4* | on | 0.202 | 0.237 | 0.002 | 0.440 | 0.126 | 0.000 | 0.039 | 0.057 | 0.000 | 0.604 | 0.100 | 0.250 | 2.42 | outside (not flown in continuum) | 0.586 | 0.530 | - |
| 25 | 108.4* | off | 0.202 | 0.237 | 0.002 | 0.441 | 0.126 | 0.000 | 0.039 | 0.057 | 0.000 | 0.605 | 0.100 | 0.290 | 2.09 | outside (not flown in continuum) | 0.587 | 0.530 | - |

### Booster descent, body without fins

| Mach | band | face | friction | base | derived | +- | table | derived/table | verdict | hull face only | Hoerner face | with bells |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0.3 | subsonic | 0.777 | 0.053 | 0.132 | 0.962 | 0.088 | 0.959 | 1.00 | within | 1.149 | 0.962 | 0.798 |
| 0.6 | subsonic | 0.831 | 0.048 | 0.167 | 1.045 | 0.097 | 0.985 | 1.06 | within | 1.245 | 1.045 | 0.869 |
| 0.8 | transonic | 0.890 | 0.045 | 0.203 | 1.138 | 0.188 | 1.037 | 1.10 | within | 1.352 | 1.138 | 0.949 |
| 0.95 | transonic | 0.947 | 0.043 | 0.237 | 1.228 | 0.203 | 1.190 | 1.03 | within | 1.456 | 1.228 | 1.027 |
| 1.05 | transonic | 0.997 | 0.043 | 0.238 | 1.278 | 0.212 | 1.391 | 0.92 | within | 1.513 | 1.278 | 1.063 |
| 1.2 | transonic | 1.081 | 0.041 | 0.208 | 1.331 | 0.225 | 1.474 | 0.90 | within | 1.566 | 1.331 | 1.086 |
| 1.5 | supersonic | 1.766 | 0.039 | 0.167 | 1.971 | 0.357 | 1.420 | 1.39 | outside | 1.650 | 1.424 | 1.123 |
| 2 | supersonic | 1.910 | 0.034 | 0.125 | 2.069 | 0.384 | 1.457 | 1.42 | outside | 1.722 | 1.535 | 1.152 |
| 3 | supersonic | 2.023 | 0.033 | 0.083 | 2.140 | 0.405 | 1.783 | 1.20 | within | 1.772 | 1.697 | 1.168 |
| 5 | supersonic | 2.084 | 0.021 | 0.050 | 2.155 | 0.313 | 1.911 | 1.13 | within | 1.777 | 1.699 | 1.154 |
| 8 | supersonic | 2.106 | 0.012 | 0.022 | 2.140 | 0.316 | 1.911 | 1.12 | within | 1.757 | 1.679 | 1.129 |
| 15 | supersonic | 2.115 | 0.005 | 0.006 | 2.126 | 0.317 | 1.911 | 1.11 | within | 1.742 | 1.663 | 1.110 |
| 25 | supersonic | 2.118 | 0.002 | 0.002 | 2.122 | 0.318 | 1.911 | 1.11 | within | 1.737 | 1.659 | 1.105 |

### Booster descent, body + fins

| Mach | fin CA tail first (gross) | fin CA nose first (gross) | +- | fin CA table | total, 12.63 | total, 13.5 | +- | table total | verdict (12.63 / 13.5) | flights C_D at 261 t (sd, n) | verdict against flights (12.63 / 13.5) | beta 12.63 | beta 13.5 | beta tables | beta flights (sd) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0.3 | 0.233 | 0.213 | 30 % | 0.821 | 1.101 | 1.111 | 0.097 | 1.448 | outside / outside | - | no flight data / no flight data | 3726 | 3694 | 2833 | - |
| 0.6 | 0.244 | 0.224 | 50 % | 0.843 | 1.191 | 1.201 | 0.122 | 1.487 | within / within | - | no flight data / no flight data | 3446 | 3417 | 2760 | - |
| 0.8 | 0.351 | 0.334 | 50 % | 0.900 | 1.347 | 1.361 | 0.215 | 1.573 | within / within | - | no flight data / no flight data | 3046 | 3014 | 2608 | - |
| 0.95 | 0.392 | 0.375 | 50 % | 0.962 | 1.461 | 1.477 | 0.234 | 1.763 | within / within | - | no flight data / no flight data | 2808 | 2777 | 2327 | - |
| 1.05 | 0.354 | 0.440 | 50 % | 0.988 | 1.489 | 1.504 | 0.237 | 1.979 | outside / outside | 2.06 (-, 1) | outside / outside | 2755 | 2728 | 2073 | 1995 (-) |
| 1.2 | 0.346 | 0.439 | 50 % | 1.000 | 1.536 | 1.551 | 0.248 | 2.069 | outside / outside | 2.28 (0.14, 4) | outside / outside | 2670 | 2646 | 1983 | 1804 (108) |
| 1.5 | 0.293 | 0.402 | 50 % | 0.983 | 2.145 | 2.157 | 0.367 | 2.006 | within / within | 1.71 (0.17, 3) | outside / outside | 1912 | 1902 | 2045 | 2410 (232) |
| 2 | 0.177 | 0.308 | 30 % | 0.900 | 2.174 | 2.182 | 0.385 | 1.993 | within / within | 1.91 (0.18, 6) | within / within | 1887 | 1881 | 2059 | 2162 (201) |
| 3 | 0.179 | 0.318 | 30 % | 0.800 | 2.246 | 2.253 | 0.407 | 2.259 | within / within | 2.28 (0.18, 23) | within / within | 1827 | 1821 | 1816 | 1809 (141) |
| 5 | 0.169 | 0.312 | 30 % | 0.750 | 2.256 | 2.263 | 0.314 | 2.357 | within / within | 2.34 (-, 1) | within / within | 1819 | 1813 | 1740 | 1751 (-) |
| 8 | 0.165 | 0.309 | 30 % | 0.750 | 2.238 | 2.245 | 0.317 | 2.357 | within / within | - | no flight data / no flight data | 1833 | 1828 | 1740 | - |
| 15 | 0.161 | 0.306 | 30 % | 0.750 | 2.222 | 2.229 | 0.319 | 2.357 | within / within | - | no flight data / no flight data | 1846 | 1841 | 1740 | - |
| 25 | 0.159 | 0.305 | 30 % | 0.750 | 2.217 | 2.224 | 0.319 | 2.357 | within / within | - | no flight data / no flight data | 1850 | 1845 | 1740 | - |

### Mesh factors k (per unit cp_max, on 63.6 m^2)

- Tail first, no bells: booster/hull 0.9428, booster/skirt 0.1657, booster/strakes 0.0437, fin 0.0396
- Tail first, drawn bells: booster/hull 0.2645, engine 0.2207, booster/skirt 0.0700, booster/strakes 0.0437, fin 0.0396
- Stack nose first, flaps 90, exposed (wake range): booster/skirt 0.1393, ship/hull 0.1186, fin 0.0626, booster/top_truss 0.0480, flap 0.0419, ship/hinge_covers 0.0135, ship/catch_pins 0.0121, ship/ports 0.0065, booster/fin_fairings 0.0057, booster/hot_stage 0.0049, booster/strakes 0.0047, ship/chine 0.0016, ship/raceway 0.0009, booster/raceway 0.0002
- Same, plain first hit (not used): ship/hull 0.1186, booster/skirt 0.0635, fin 0.0587, booster/top_truss 0.0423, flap 0.0353, ship/hinge_covers 0.0130, ship/catch_pins 0.0121, ship/ports 0.0065, booster/strakes 0.0025, ship/raceway 0.0009, ship/chine 0.0009, booster/raceway 0.0000, booster/fin_fairings 0.0000
- Stack nose first, flaps 0, exposed: booster/skirt 0.1393, ship/hull 0.1152, fin 0.0646, booster/top_truss 0.0431, flap 0.0346, ship/hinge_covers 0.0142, ship/catch_pins 0.0121, ship/ports 0.0065, booster/fin_fairings 0.0057, booster/hot_stage 0.0049, booster/strakes 0.0047, ship/chine 0.0011, ship/raceway 0.0009, booster/raceway 0.0002
- Ship nose first, flaps 90, exposed: ship/hull 0.1186, flap 0.0420, ship/hinge_covers 0.0135, ship/catch_pins 0.0119, ship/ports 0.0064, ship/chine 0.0016, ship/raceway 0.0010
- Solid load per fin, nose first, m^2: 0 deg 1.295, 90 deg 1.376, 180 deg 1.311
- Fin, per gross area: tail first 0.086, nose first 0.165; aft wall edge mean cos^2 0.322; mean cell depth, flat forward face 0.500 m, scooped aft face 0.321 m
- Hot-stage open band 2.56 m

### Parts term at Mach 1.2, per part: k with the layer credit, of k without

- booster/skirt: 0.0906 of 0.1393
- ship/hinge_covers: 0.0129 of 0.0135
- ship/catch_pins: 0.0118 of 0.0121
- ship/ports: 0.0065 of 0.0065
- booster/fin_fairings: 0.0033 of 0.0057
- booster/strakes: 0.0032 of 0.0047
- ship/chine: 0.0009 of 0.0016
- ship/raceway: 0.0007 of 0.0009
- booster/raceway: 0.0001 of 0.0002
- Open fraction 0.825
- Flight 14, stack to T+160 s: peak q 38.9 kPa at T+63 s (Mach 1.36, 9.2 km). Drag / thrust there: derived 2.7 %, table 1.6 %; their difference is 0.23 m/s^2 of acceleration. Velocity lost to drag: derived 30 m/s, table 17 m/s.

## The stack's table
`config/aero/starship_stack_ascent.csv` is this tool's derived total since 2026-11-01 (user's OK "Anbauteile ergänzen"; orchestrator's
decision after the review: the total, not old table + fins + parts, which counts the old table's unknown content twice). Written by
`python tools/aero_derive/derive_ascent_descent.py --write-stack-table`; [E]; no band fitted. Mach 0 holds the Mach 0.3 value. The last row is
Mach 5, held beyond: flight 14 stages at about Mach 5.4 (T+160 s); Mach 6 is not a stack state on that flight (the dials reach it high
up, outside the continuum).

| Mach | old, on | new, on | +- | new / old | old, off | new, off | +- | new / old |
|---|---|---|---|---|---|---|---|---|
| 0.3 | 0.300 | 0.424 | 0.073 | 1.41 | 0.380 | 0.536 | 0.082 | 1.41 |
| 0.6 | 0.300 | 0.447 | 0.095 | 1.49 | 0.380 | 0.589 | 0.106 | 1.55 |
| 0.8 | 0.330 | 0.531 | 0.123 | 1.61 | 0.410 | 0.704 | 0.136 | 1.72 |
| 0.95 | 0.450 | 0.790 | 0.174 | 1.76 | 0.550 | 0.992 | 0.186 | 1.80 |
| 1.05 | 0.580 | 1.104 | 0.286 | 1.90 | 0.680 | 1.307 | 0.295 | 1.92 |
| 1.2 | 0.620 | 1.049 | 0.187 | 1.69 | 0.720 | 1.226 | 0.197 | 1.70 |
| 1.5 | 0.570 | 0.965 | 0.172 | 1.69 | 0.660 | 1.107 | 0.179 | 1.68 |
| 2 | 0.500 | 0.862 | 0.130 | 1.72 | 0.580 | 0.969 | 0.134 | 1.67 |
| 3 | 0.400 | 0.829 | 0.127 | 2.07 | 0.470 | 0.900 | 0.129 | 1.91 |
| 4 | 0.350 | 0.804 | 0.124 | 2.30 | 0.410 | 0.857 | 0.125 | 2.09 |
| 5 | 0.325 | 0.797 | 0.120 | 2.45 | 0.380 | 0.839 | 0.121 | 2.21 |

Who flies what: the booster stage's `ascent_drag_table` is the stack's, and the lowest stage with a table sets the vehicle's
(`Vehicle::rebuild_aero`), so `starship_block3`, `starship_tanker`, `starship_depot`, `starship_hls_launch` fly the new table in the stack.
The ship stages (`starship_block3`, by `from =` also the tanker; `starship_hls`, also `starship_hls_launch`; `starship_depot`) carry
`config/aero/ship_ascent.csv`, the old values unchanged. Falcon 9 has its own files, untouched. The booster alone nose first keeps the
stack's table: not flown beyond the first seconds after staging, where dCD 0.62 at q 130-470 Pa gives 0.01-0.05 m/s^2 at 400 t, and tail first it
flies `super_heavy_descent.csv`. Plan cache: `config_contents_hash` covers the config directory, so old plans miss.

## Verdicts per Mach band

| Case | Mach | Verdict | Reason, as far as known |
|---|---|---|---|
| Ascent stack, both columns, against the old table | 0.3-5 | **outside**, 1.4-2.5: the reason for the new table | Fins 0.13-0.26 (wall 0.012 m) and other drawn parts 0.12-0.20 are the excess. The old table is a generic trend from Saturn V / Shuttle-class shapes (its header); those carry fins and protuberances, so what it holds of them is unknown and it was replaced, not added to |
| Ascent stack | 8-25 | outside, not a flown state | The stack stages at about Mach 6. Friction at Kn 0.01 is laminar and at the end of its range |
| Ascent ship alone | 5 | 1.29 on, **just outside** (0.419 against 0.325 +- 0.089 combined); 1.14 off, within | Went from within to just outside when the wake range raised the flap load; left as it is, Task 5 re-determines the ship |
| Ascent ship alone | 8-25 | outside, ratio 2.1-4.1, not flown in continuum | Friction near Kn 0.01; at the flown altitude the core's free-molecular blend dominates and the table's weight is small |
| Descent body | 0.3-1.2 | within (0.90-1.10) | Hoerner blunt body; subsonic is unmeasured on both sides |
| Descent body | 1.5, 2 | **outside** (1.39, 1.42) | Newtonian loads the flat face with full stagnation pressure, and the protrusions (k 0.21). Hoerner's face gives 1.42 / 1.54 against 1.42 / 1.46, so the method jump at Mach 1.5 is most of it |
| Descent body | 3-25 | within (1.11-1.20) | Hull face alone: 1.77 at Mach 3-5 against 1.78-1.91 |
| Descent total | 0.6-0.95, 1.5-25 | within the table | Body high and fins low compensate from Mach 1.5 up: derived fin CA 0.16-0.29 against the table's 0.75-0.98. The agreement of the sum does not confirm the split |
| Descent total | 0.3, 1.05, 1.2 | outside the table (1.10 against 1.45; 1.49 against 1.98; 1.54 against 2.07) | Fin form changes at Mach 1 and the blunt-face trend is the least certain band; Mach 0.3 is unmeasured on both sides |
| Descent total against flights | 1.05, 1.2 | **outside**: 1.49, 1.54 against 2.06 (one bin), 2.28 | Derived too low in the band flagged least certain |
| Descent total against flights | 1.5 | **outside**: 2.15 against 1.71 | As descent body at 1.5 |
| Descent total against flights | 2 | within: 2.17 against 1.91 | |
| Descent total against flights | 3, 5 | within: 2.25, 2.26 against 2.28, 2.34 (one bin) | Beta 1827 / 1819 against 1809 / 1751 kg/m^2; Mach 3 is the densest band (23 bins) |

What the flights say about the ascent: **they cannot resolve its Cd.** At peak q (38.9 kPa, T+63 s) the derived and table drag differ by
0.23 m/s^2, 1.1 % of thrust, less than the thrust change of one engine out (flight 14 flew 32 of 33). Velocity lost to drag up to T+160 s
is 30 m/s with the new table and 17 m/s with the old, with the mass history assumed. `late_ascent_fit.py` works where drag is under 0.5 % of thrust and
does not separate it.

## Sensitivities
- Fin area 13.5 (config until Task 4) against 12.63 m^2, derived fin CA: descent total +0.010 to +0.033 (at most 2.0 %), beta -8 to -59 kg/m^2.
- Flaps at d = 0 instead of 90 in ascent (note only, d = 90 is the real pose): stack -0.006 to -0.011 (with the wake range folded flaps
  hide little). Flaps at d = 90 carry 0.045-0.075 in the stack at Mach 0.3-5.
- Nose 17 m (the physics until Task 5b) instead of 13.4 m (drawn): -0.07 to -0.15 at Mach 0.95-5, nothing below.
- Wake range 5 / 10 / 20 heights: parts k 0.186 / 0.185 / 0.178, skirt 0.139 / 0.139 / 0.133, fins 0.0626 / 0.0626 / 0.0618. Insensitive.
- Layer credit: parts term 0.115-0.202 against 0.168-0.310 without it (power on, Mach 0.3-5).
- Grid fin wall (0.012 m, range 0.006-0.044): the drawn upper bound 0.04 gave the stack 0.33-0.45 for today's 0.13-0.26. The stack's total at Mach 0.3 / 2 / 5: 0.640 / 1.090 / 0.942
  at 0.04, 0.424 / 0.862 / 0.797 at 0.012. Scooped forward face (not seen in a photo, drawn flat) would lower it further.
- Drawn bells in the pure Newtonian descent: body 1.12-1.17 at Mach >= 1.5, a lower bound (see Method).
- Hull face only against all drawn parts, descent Mach >= 1.5: 1.65-1.78 against 1.97-2.16.

## Limits
- Book constants (Hoerner, Fleeman, Idel'chik) were recalled, not re-read from a printed copy; the bands are wide for this and marked [E].
- Layer credit: incompressible thickness and constant density. At high Mach the real layer is thicker and less dense near the wall, so the credit is on the
  small side at Mach 3-5. Rods, strakes and skirt have no wind-tunnel reference here.
- Hull friction is hydraulically smooth (+-20 %). Fully rough with k 1 mm on 124 m would give about 0.0030 against 0.0016 at Mach 0.3, recalled; so
  the 0.084 can be low by more than its band. The 33 drawn exits are 85 % of the disc and are not checked against the rings; effect on base
  drag small (0.02 power on).
- No interference between fins and hull, no real-gas cp_max above Mach ~8. The fin's started form jumps at Mach 1 (Idel'chik to Newtonian):
  tail first it falls 25 % across Mach 1 (0.79 to 0.55), nose first 6 % (0.75 to 0.71). So `grid_fin.csv` must not be reviewed against this step in Task 4.
- Mach >= 1.5 descent body loads the skirt and strake ends, which stand in the base ring's bubble (k 0.209); most of the step 1.33 to 1.97.
- Grid fin wall thickness 0.012 m (`geometry.DERIVATION_WALL_M`; 0.006-0.044, factory photo, may be a flange) sets the open fraction 0.825; the drawn `wall_m` stays 0.04 m. Mesh walls have no thickness.
- Stack rows at Mach >= 8 and descent rows at Mach >= 5 are not measured states: descent friction there uses a nominal 30 km.

## Grid fin authority: depth and cells (Task 4b, flown since Task 4c)
`tools/aero_derive/grid_fin_lattice.py` writes `screenshots/vehicle-physics/grid_fin_lattice.{csv,md}` (all rows); `--write-table` writes `config/aero/super_heavy_grid_fin.csv`,
which the Super Heavy flies since the user's OK of 2026-11-01. "table" below is the generic `grid_fin.csv`, which Falcon 9 keeps unchanged. `ca_axial` of the new table is the generic's on the
finer grid, not derived: the descent total the flights fixed stays. Photo measurement: `tools/structures/measure/fin_rim_depth.py`.

**Secured geometry** (photo = `padb_chopsticks_around_booster_top_2026-03.jpg`, both a catch and the plain fin at one scale, 41 px/m, 17 deg elevation):

| Quantity | Nominal | Range | Evidence, which reading wins |
|---|---|---|---|
| Pitch | 0.35 m | 0.33-0.37 | STL 0.350 wins over the f12 reading 0.47 and the earlier 0.49. Rim tooth spacing in the photo 19-20 px = 0.46-0.50 m in the image, about 0.50-0.53 m with the rim's foreshortening (view 21-27 deg off the fin, from its position) = p sqrt 2. A pitch of 0.47 needs 0.665 m, about 25 px. The rim tooth count and the 102.9 cells of the fitted image agree. The f12 frame is the lowest resolution of the three |
| Depth at the cusp tips | 0.45 m | 0.40-0.50 | Photo: catch 0.48 (0.465 with the fin about 3 % nearer, assumed), plain 0.42, +-0.03 each. STL 0.40. The photo wins over the STL and over the drawn 0.50, within the errors of both |
| Rim scallop | 0.15 m | 0.10-0.20 | Photo: crowns 0.28-0.30 m below the flat face. STL 0.09 (the STL does have scallops; the earlier "STL lacks them" is wrong). Drawn `edge_tooth_depth_m` 0.22 is high |
| Interior scoop | 0.22 m | 0.09-0.30 | STL 0.09, hearts fit 0.30, factory photo about 0.28 (sag / oblique view by eye, weak) |
| Mean wall chord | 0.32 m | 0.25-0.40 | This is what loads. STL wall area / length 0.323, drawn 0.321, rim 0.36-0.38 |
| Wall | 0.012 m | 0.006-0.044 | Factory photo: bright edge lines median 4 px (2-6) at about 400 px/m = 0.010 m. STL 0.044 / wide variant 0.060 are print widths (guide: 0.2 / 0.4 mm nozzle variants, walls 0.44 / 0.60 mm), so not evidence. The edge may be a flange: the range keeps the old upper bound |

**Catch fins are not deeper.** The `-catch` STLs have the same 0.40 m lattice (identical wall profile, y 0.30-0.70); the 0.7 m is only the bar at the pylon, y 0-0.30. In the photo the catch fin's tip 0.465-0.48 and the plain fin's 0.42 differ by 0.05 +-0.05 m, the crowns by -0.02 (opposite sign), mean rim 0.38 / 0.36. 0.7 m would be 28 px; measured 16-19. Identity: the right fin points away from the tower (the 90 deg fin, config), the front fin stands over the arm (0 / 180). The layout is unequal by azimuth only, no fin differs in depth or station.

**Method**, sources and what is from memory: docstring of the tool. Weinig's cascade, Prandtl's box factor, Kantrowitz's limit, the thin-plate stall incidences and NACA 1135 are recalled from memory, with the first checked by its two limits and the last by chart values in the tests. The reflection law and the momentum closure are derived in the tool. Web check 2026-11-01: [Miller 1993 AIAA paper, abstract](https://www.scribd.com/document/514793588/washington1993) (distinct choking and shock Mach numbers, isentropic + 2-D airfoil theory) and [IISc thesis](https://etd.iisc.ac.in/handle/2005/563) (too deep a cell goes negative by reflected shocks) support the structure; no numbers were taken. **No grid fin measurement is in the repo; the table's "~0.05 /deg" could not be checked at its source, nor its reference area.** Errors are judgments.

| Mach | regime | cn_alpha /rad, 10 deg secant | model | geometry band | table | ratio | limit deg | table |
|---|---|---|---|---|---|---|---|---|
| 0 | subsonic | 1.46 | 25 % | 1.27-1.53 | 2.90 | 0.50 | 32 +-10 | 25 |
| 0.7 | subsonic | 1.58 | 25 % | 1.21-1.61 | 3.10 | 0.51 | 27 +-8 | 24 |
| 0.9 | choked | 1.58 | 40 % | 1.24-1.64 | 2.20 | 0.72 | 22 +-7 | 20 |
| 1.1 | choked | 1.69 | 40 % | 1.36-1.75 | 1.60 | 1.06 | 18 +-5 | 20 |
| 1.4 | choked | 1.79 | 40 % | 1.44-3.56 | 2.10 | 0.85 | 18 +-5 | 24 |
| 1.5 | started from 1.42 | 2.94 | 35 % | 1.48-3.45 | 2.07 | 1.42 | 18 +-5 | 24 |
| 2 | started | 1.94 | 20 % | 1.28-2.88 | 1.90 | 1.02 | 31 +-13 | 26 |
| 3 | started | 1.20 | 20 % | 0.79-1.78 | 1.40 | 0.86 | 44 +-21 | 28 |
| 5 | started | 0.72 | 30 % | 0.47-1.07 | 0.90 | 0.80 | 45 +-14 | 30 |
| 10 | started | 0.41 | 30 % | 0.27-0.61 | 0.60 | 0.69 | 45 +-14 | 30 |
| 25 | started | 0.28 | 30 % | 0.18-0.41 | 0.50 | 0.56 | 45 +-14 | 30 |

Verdict: Mach 1-5 within the errors, including peak q. **Below Mach 0.9 the table is about twice the derivation and outside its error.** The bound does not rest on the fitted parts: a guided lattice turns at most its own mass flow, 2 /rad per frontal area, and a solid plate of this planform has 2.24 /rad (Helmbold, AR 1.63). The table's 2.9-3.1 passes both, unless its source used another reference area (hypothesis). Finite-stack end effects could raise the derived value toward 2, not 3. Above Mach 7 the table is 1.4-1.8 x high, at a q where fins hardly matter.

**Depth.** Above the ideal Mach (c = p sqrt(M^2 - 1), Mach 1.4 for the nominal chord) the slope is proportional to chord: tip 0.40 / 0.45 / 0.50 / 0.70 at Mach 2.63 gives 1.24 / 1.39 / 1.54 / 2.16. Subsonic it saturates at the momentum limit: 1.49 / 1.51 / 1.53 / 1.56 at Mach 0.5. Scoop counts as much as tip: chord 0.45 without scoop 1.96 at Mach 2.63. Limit: unaffected.
**Pitch.** 0.33 / 0.37 / 0.47 at Mach 2.63: 1.47 / 1.31 / 1.03; Mach 0.5: 1.52 / 1.50 / 1.44.
**Wall.** Slope above the start unchanged, but it sets where the cells start: wall 0.006 / 0.012 / 0.02 / 0.044 = Mach 1.26 / 1.42 / 1.62 / 2.46. At 0.044 the cell at peak q is just started and unstarts at 18 deg fin incidence: force 0.62 against 1.16 MN. So the old upper bound would halve the authority at peak q; the wall is the least secured and second most important input.

**Authority**, 3 fins, 30 deg commanded, roll r 7.19 m, pair and single fin about lever 44.1 m (from Task 4's 94.3 MN m). Derived with the secant at the angle reached, since the supersonic load grows faster than sin(alpha). MN m:

| Point | q kPa, Mach | | angle deg | fin force MN | roll | pitch (pair) | yaw (single) |
|---|---|---|---|---|---|---|---|
| f13 peak q | 106.7, 2.63 | table | 27.3 | 0.98 | 21.1 | 86.3 | 43.1 |
| | | derived +-20 %, all geometry corners 0.46-1.72 MN (wall 0.044 at the low end) | 30 | 1.16 | 25.0 | 102 | 51.2 |
| f14 peak q | 101.0, 2.44 | table | 26.9 | 0.97 | 20.9 | 85.5 | 42.7 |
| | | derived +-20 % | 30 | 1.19 | 25.7 | 105 | 52.5 |
| Landing burn ignition | 39.0, 0.77 | table | 22.6 | 0.53 | 11.4 | 46.5 | 23.2 |
| | | derived +-40 % | 25.4 | 0.33 | 7.2 | 29.4 | 14.7 |
| Ascent max q, not commanded | 38.9, 1.36 | table | 23.5 | 0.40 | 8.6 | 35.1 | 17.5 |
| | | derived +-40 % | 18.1 | 0.27 | 5.8 | 23.9 | 11.9 |

At small deflection peak q is 0.88 of the table (1.39 / 1.585, Mach 2.63); at 30 deg 1.19. Task 4's 94.3 MN m was the slope without limit. No flight data separates these: the telemetry replay covers the fall without commanded fins. Effect in flight not run (no core change, Task 6).

**Axial force.** Depth hardly matters: tip 0.40 to 0.50 moves the CD term 3 x 12.63 / 63.617 x CA by at most -0.004 to +0.002 nose first, -0.008 to -0.001 tail first (friction up, the entry loss down, nearly cancelling), the excluded 0.70 at most +0.005 / -0.009. The wall dominates: at the nominal lattice against today's drawn (tip 0.5, wall 0.04) the **ascent fin term** falls by 0.14-0.23 (Mach 0.3: 0.337 to 0.124; Mach 2: 0.414 to 0.185; Mach 5: 0.329 to 0.184) and the derived **descent** term by 0.04-0.24 (Mach 0.8: 0.459 to 0.216; Mach 3: 0.186 to 0.112). The stack table's fin term (Task 2b) was built on wall 0.04, so it is high by that much if the wall reading holds. Descent table total does not move (fixed against flights); the derived fin CA tail first falls further below `ca_axial` (Mach 5: 0.18 against 0.75), which widens the open split. The couple in the core follows `ca_axial`, unchanged.

**Open:** the wall (a sharper photo of a wall edge would settle it). Below Mach 0.9 the derivation stays 0.5 of the generic table, outside its error, and the flights do not measure it. Flight effect on the catch: Task 6. Not done: CFD, flight effect, fin-specific literature values.

## Grid fins in the core (Task 4)
Config: `area_m2` 12.63 (gross, drawn), `radius_m` 7.19 (outline centroid: hull 4.5 + root gap 0.15 + 2.544), `station_m` 64.0: the outline's
centroid lies on the hinge (z 0.000) and the plate is centred on it in depth (x +-0.25), so no disagreement. Falcon 9's fins untouched.

- **Lever and attitude:** `fin_transform` (rotation about the azimuth, then about the radial) gives cell axis x cos d - t sin d and normal
  t cos d + x sin d, the core's `grid_fin_force` normal, to 5e-17, at all three azimuths; at d = 0 the cells are axial. The axes agree.
- **Descent table:** `super_heavy_descent.csv` = old body + 3 x 0.87 m^2 x `ca_axial` / 63.617 m^2 on the union of both Mach grids, so the
  total against the flights did not move (largest change 4.3e-5, rounding; `[aero][grid_fins]` checks < 1e-3 over Mach 0-25). `ca_axial`
  is not re-fitted.
- **Open:** derived fin CA tail first is 0.24-0.79 against the table's 0.75-1.0, and the sum body + fins agrees only because the body is
  high where the fins are low. The split between them remains undetermined; moving it would change the moment below, not the total.

### The unbalanced fin's moment
Fins 0 and 180 cancel, fin 90 stands at body +z: its axial drag makes a moment about +y, the axis on which the fins 0 and 180 act. Peak q from
`tools/telemetry/peak_dynamic_pressure.py`; authority from the core with the new area and lever, at the measured state (261 t, CG 21.86 m;
stack 3951 t at T+63 s, CG 53.38 m).

| Phase | q kPa | Mach | CA | Force kN | Moment MN m | Authority about y, MN m | Share |
|---|---|---|---|---|---|---|---|
| Tail first, flight 13 peak (T+372 s, 11.2 km) | 106.7 | 2.63 | 0.837 table | 1128 | 8.11 | fins 94.3 | **8.6 %** |
| Same, derived CA | 106.7 | 2.63 | ~0.37 | 499 | 3.6 | fins 94.3 | 3.8 % |
| Tail first, flight 14 peak (T+382 s, 10.6 km) | 101.0 | 2.44 | 0.856 table | 1092 | 7.85 | fins 94.5 | **8.3 %** |
| Tail first, against RCS (no engine running) | 106.7 | 2.63 | 0.837 | 1128 | 8.11 | RCS 0.587 | 1380 %, the fins must hold it; passes the RCS at q about 8.6 kPa |
| Landing burn ignition, simulated (0.62 km, 260 m/s) | 39 | 0.77 | 0.88 | 432 | 3.1 | gimbal 13 engines 214 | 1.5 % |
| Ascent, max q flight 14 (T+63 s) | 38.9 | 1.36 | 1.00 table | 491 | 3.53 | gimbal 480 (vacuum thrust) | 0.74 % |
| Same, derived nose-first CA | 38.9 | 1.36 | 0.725 | 356 | 2.56 | gimbal 480 | 0.53 % |

Decision from the numbers (2026-11-01, Task 6): **off by default.** The rows above use the wall 0.04 m; with the flown derivation wall 0.012 m the tail-first
term at Mach 3 is 0.112 on 63.62 m^2 for three fins, so CA 0.19 per fin against the table's 0.837: about 1.8 MN m, 1.8 % of the 102 MN m of authority, under the 2 % threshold. The table's
CA overstates it about 4 x (the descent total keeps that CA only because the sum was fixed by the flights). The ascent is below 2 % by any CA.

`FlightBody` (`grid_fin_drag_couple`, fidelity flag `FlightEnvironment::grid_fin_drag_couple`, default off; set through the environment only)
with the flag on adds the moment of each fin's share of the axial force (`aero::grid_fin_axial_force_n`) at its centre of pressure. The force itself stays in the axial
curve, because the planners read that curve (`booster_return_prediction`, `descent_model`): force on the axis + this couple is the same
wrench as the drag at the fins with it taken out of the curve, and the total cannot move. A symmetric layout gets exactly zero (Falcon 9
bit-identical, printed state compared in hex). The guidance has no feed-forward for this moment; the attitude loop takes it as a disturbance.

### Why fins 0 and 180 load below fin 90 (nose first)
Solid load 1.295 / 1.376 / 1.311 m^2 (0 / 90 / 180). Occlusion, not a bug, and not the catch pins the review suspected: removing `ship/catch_pins`
changes nothing. With hulls and fins alone the three are 1.451 / 1.439 / 1.463. Removing the flaps gives 1.388 / 1.376 / 1.400: fin 90 is unchanged and
0 / 180 rise above it. So the deficit is the wake of the flaps (hinge azimuths 0, 180, 20, 160), upstream of those two fins and not of fin 90. The wake rule takes a part's
span as its height, which overstates a thin flap's wake (see `exposure.py`), so the 5-6 % is an upper value. The remaining difference between 0 and 180 (1 %) is the
drawn mesh, partly the bearing on those fins.

## Ship entry (Task 5)
`python tools/aero_derive/derive_ship_entry.py --trim <csv>` (pairs: hidden test `[trim_dump]`, `STARSIM_TRIM_CSV`); `--write-hull-cp` writes
`config/aero/ship_hull_cp.csv`. Modified Newtonian first hit above Mach 3, Hoerner below. Flown since Task 5b: nose 13.4 m, hull station table (fade Mach 1-4), `ship_ascent.csv` 0.419 / 0.433 from Mach 5. NOT flown: the crossflow row (gap below open) and the alpha. The webcast alpha supports the old table (`task-5a-report.md`):

| Quantity | Tables today | Drawn geometry | Note |
|---|---|---|---|
| Hull eta C_dc, Mach 4 / 8 / 15 / 25 | 1.50 / 1.50 / 1.52 / 1.58 | 1.07 / 1.09 / 1.10 / 1.10 +-10 % | Mean over alpha 50-90, potential term out; alpha 90 gives CN 535 m^2 = 2/3 cp_max on the 428 m^2 planform |
| Hull eta C_dc, Mach 0.3 / 0.8 / 1.2 / 2 | 1.11 / 1.30 / 1.90 / 1.70 | 0.84 / 1.20 / 1.80 / 1.50 (+-0.25-0.40) | Hoerner, recalled |
| Hull normal force station, alpha 50 / 60 / 90 | about 27 m (formula) | 25.61 / 24.97 / 23.61 m | Table from Mach 4, faded in over Mach 1-4; below Mach 1 the formula |
| Flap exposure, aft d = 30 / 40 / 60 / 90 | core 0.46 / 0.58 / 0.79 / 0.91 | mesh 0.41 / 0.65 / 0.82 / 0.85 | Forward, alpha 60: core rule against mesh within about 0.1 |

The derived hull alone at the old alpha flies 17-20 % rms too fast on flights 13, 14 (drag short). The gap between 1.07-1.10 and the flights' 1.5-1.6 is open: `ship_crossflow.csv` stays [D].

## Open
Ascent validation `[gnc][validation][ascent][slow]` with the new table ran in Task 6: green. Fin CA: see
"Grid fins in the core". No guidance feed-forward for the fin moment; Task 6 ran with the fin drag couple off and showed no need (under 2 % by the review's figure).
