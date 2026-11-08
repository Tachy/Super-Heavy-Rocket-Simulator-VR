# T9 report: atmosphere spectrum check (investigation, no model change)

Status: DONE (proposal only; nothing in config / core changed). Scripts (CPU, offline, committed):
`tools/validation/atmosphere_spectrum_check.py [--g173 ASTMG173.csv]` and `tools/validation/atmosphere_channel_fix_check.py`.
They rebuild the core's optics in numpy from `atmosphere_spectra.hpp` / `atmosphere_optics.cpp` / `earth_optics.toml`
(Rayleigh Peck & Reeder + Bucholtz King factor, 300 DU on the binned Serdyuchenko cross sections, AOD 0.1 / Angstrom 1.3 /
SSA 0.9 / g 0.8, G-173 ETR in 10 nm bins), spherical air masses per constituent. G-173 reference spectra: pvlib v0.11.1
`ASTMG173.csv` (downloaded to the scratchpad only). CCT: nearest Planckian point in CIE 1960 uv (McCamy printed too).

## Where the model's colour comes from (key finding)

The question's premise is half right: the SKY is not 3-wavelength. Sky radiance and sky irradiance are Bruneton's
precomputed-luminance mode, 15 wavelengths 360-830 nm folded with the CIE functions. Only the DIRECT BEAM (and every
transmittance: Sun light, aerial perspective) is 3-wavelength: `sun_lux = solar_rgb(lambda_c) x k_c x T(lambda_c)` with
T read at 680 / 550 / 440 nm (`spectra::lambda_*_nm`, `atmosphere_precompute.cpp` line ~926, `atmosphere_tables.cpp`).
The red channel is the error: the sRGB red channel's sunlight-weighted response sits at an EQUIVALENT wavelength of
628 nm (green 546, blue 444; stable within 1 nm from Sun 90 to 15 deg), so T(680) overstates red transmittance (T_eff/T
0.945 at 48 deg, 0.855 at 15 deg, 0.68 at 5 deg) and the direct beam comes out too warm and, at low Sun, too bright.

Replica check: the script's 3-channel direct beam at Sun 48 deg is 4934 K / B/R 0.612 / 100 512 lx normal; the game
probe (t5b fix 2) printed 4936 K / 0.612 / 100 446 lx. Global with the model's own sky (15 834 lx, B/R 1.789): 5503 K
(probe 5510 K). The replica reproduces the model.

## (a) Spectral representation (model optics unchanged)

CCT in K. "full" = 1 nm spectrum through the same optics; "3ch" = the core now; "15 wl" = Bruneton's 15-wavelength sum.
Sky and global from SPCTRAL2 (Bird & Riordan 1986) as a spectral sky proxy (differences only; the core's sky is its own
multiple scattering - proxy sky share 16.3 % vs the core's 17.5 % at 48 deg, CCT 10 520 vs 10 500 K).

| Sun elev | direct full | direct 3ch (now) | direct 15 wl | sky full | sky 15 wl | global full | global now-like |
|---|---|---|---|---|---|---|---|
| 90 | 5310 | 5153 | 5302 | 11162 | 11025 | 5732 | 5570 |
| 60 | 5229 | 5052 | 5222 | 10906 | 10779 | 5698 | 5517 |
| 48 | 5134 | 4934 | 5127 | 10640 | 10522 | 5657 | 5454 |
| 30 | 4838 | 4571 | 4831 | 9986 | 9890 | 5533 | 5262 |
| 15 | 4187 | 3795 | 4179 | 9118 | 9045 | 5328 | 4917 |
| 5 | 2847 | 2369 | 2832 | 8768 | 8699 | 5995 | 5243 |

- Direct beam 3ch error: +5.8 / +7.9 / +12.1 / +24.7 / +71 mired at 90 / 48 / 30 / 15 / 5 deg (-157 / -200 / -267 /
  -392 / -478 K). Direct illuminance too high: +1.2 % at 90, +1.6 % at 48, +5.8 % at 15, +23 % at 5, +65 % at 2 deg.
- The 15-wavelength sky is fine: -1.0 mired (-120 K at 10 600 K); 15 vs 1 nm on the direct beam -0.3 mired.
- With the model's own sky at 48 deg: global 5503 K now -> 5700 K with a spectrally correct direct beam (B/R 0.756 -> 0.802).

## (b) Aerosol (Sun 48 deg, full spectrum)

| case | direct | sky | global | sky share |
|---|---|---|---|---|
| model 0.10 / 1.3 / 0.90 | 5134 | 10640 | 5657 | 0.163 |
| AOD 0.05 / 0.15 / 0.20 / 0.30 | 5224 / 5047 / 4963 / 4804 | 14353 / 9257 / 8511 / 7691 | 5696 / 5620 / 5586 / 5524 | 0.12 - 0.33 |
| Angstrom 0.5 / 1.0 / 1.6 | 5247 / 5176 / 5091 | 9378 / 10126 / 11213 | 5687 / 5668 / 5645 | 0.164 |
| SSA 0.95 / 0.97 | 5134 | 10481 / 10422 | 5666 / 5669 | 0.168 / 0.170 |
| coastal 0.12 / 1.0 / 0.95 | 5149 | 9349 | 5663 | 0.187 |
| rural 0.08 / 1.3 / 0.95 | 5169 | 11415 | 5679 | 0.149 |
| no aerosol | 5317 | 40535 | 5739 | 0.069 |

Climatology [S, literature from memory, to be confirmed if the user wants a change]: AERONET / MODIS background AOD550 ~0.1-0.2
(global ocean ~0.13, land ~0.19, Remer et al. 2008), Angstrom ~1.2-1.5 continental, 0.5-1.0 maritime / coastal; SSA
(550) 0.92-0.98 for clean continental, oceanic and urban-industrial, 0.88-0.94 for biomass smoke (Dubovik et al. 2002,
J. Atmos. Sci. 59, 590). The current set lies inside that range. Over the whole range the global CCT moves by only
-130 / +40 K; aerosol is NOT the cause of the warm daylight. (It strongly sets the sky's own colour and the sky share.)

## (c) Ozone (Sun 48 deg)

0 / 250 / 300 / 350 DU: direct 5003 / 5112 / 5134 / 5156, global 5492 / 5629 / 5657 / 5686 K. 300 DU [S] is right
(subtropical climatology ~270-300 DU): +-25 K per 50 DU, not a lever. The Chappuis band is included (it RAISES the CCT).

## (d) Solar spectrum

G-173 ETR in the core's 10 nm bins 5920 K vs the full-resolution G-173 ETR 5933 K (binning -0.4 mired). Leverage of a
different TOA spectrum: a slope of +-1 % per 100 nm moves TOA by -+70 K and global by -+65 K; the published TOA
spectra (G-173 ETR / Gueymard 2004, Thuillier 2003, TSIS-1 HSRS) differ in the visible by ~1-2 % without a large slope
[E]. Not a lever.

## (e) Missing gases and a measured standard

- Water vapour / O2 bands (taken from the G-173 direct / continuum ratio above 680 nm, 0.92 at 680-740, 0.80 at 750-770 nm)
  move CCT by +2 to +6 K (the CIE functions barely weigh them). Not needed.
- ASTM G-173-03 (SMARTS 2.9.2, AM 1.5 = Sun 41.8 deg, rural AOD500 0.084, 1.42 cm H2O, 0.34 atm-cm O3): direct +
  circumsolar 5232 K (B/R 0.680), global 37 deg tilt 5505 K, ETR 5933 K. Our optics at those conditions: full spectrum
  5148 K, 3ch 4914 K. The full-spectrum model is within 3 mired of the standard; the 3ch beam is 12 mired off.
- Against measurements (ranges as in the brief / t5b, [S] there): direct beam 5000-5800 K at AM 1-1.5 -> full model
  5134-5310 K (inside), now 4934-5153 K (below at AM 1.35); noon global 5500-6500 K -> full model 5650-5730 K at
  48-90 deg with the core's sky (inside, lower half), now 5510 K (edge). The remaining distance to D65 is physical for a
  horizontal plate under a clear sky with AOD 0.1 (bluer readings need a cleaner air / a larger sky share).

## Proposal (needs the user's OK; nothing committed)

1. RECOMMENDED - channel wavelengths of the transmittance table 680 / 550 / 440 -> 628 / 546 / 444 nm [D: equivalent
   wavelengths of the linear sRGB channels under the G-173 TOA Sun, `atmosphere_channel_fix_check.py`], i.e. the three
   constants in `atmosphere_spectra.hpp` (plus the comments in `atmosphere_scattering.hpp` / `atmosphere_tables.hpp`).
   `solar_illuminance_rgb` and `sun_radiance_to_luminance` follow from the same constants, so the TOA Sun colour is
   unchanged by construction; the sky tables (15-wavelength passes) are unchanged; only the transmittance table and the
   input hash change (one recompute, 61 s; cache re-shipped). Expected: direct beam within 1.2 mired of the full spectrum
   down to 15 deg (4934 -> 5135 K at 48 deg, 3795 -> 4208 K at 15 deg, 2369 -> 2960 K at 5 deg vs 2847 full), direct
   illuminance error +1.6 % -> 0.02 % at 48 deg and +23 % -> +2.5 % at 5 deg, global noon 5510 -> ~5700 K (white-balance
   illuminant B/R 0.758 -> ~0.80). Aerial perspective improves too (near-horizontal 14 km path: red T 0.444 now / 0.399
   new / 0.409 exact). Residual: -13 mired at 5 deg, -41 at 2 deg (Sun at the horizon, a few hundred lux).
   Runtime cost zero. Work: constants + a new core test (direct beam colour vs a 1 nm spectral integration through the
   reference optics, e.g. B/R within 1 % at 90 / 48 / 15 deg) + re-measure the colour-dependent game numbers
   (white_balance_test band still passes; t5b illuminant sweep, sky_radiance_check deck colours, cloud_check / Sun-beam
   rule, eye-adaptation numbers in the docs) - about one implementer task.
2. Rejected - per-channel broadband transmittance (exact for the direct beam at every air mass): the table ratio
   T(p)/T(q) for segments then breaks (sRGB weights with negative lobes: near-horizontal blue T 0.50 vs 0.18 exact).
3. Optional, independent, small - aerosol SSA 0.90 -> 0.95 [S: Dubovik et al. 2002, clean continental / maritime]:
   global +9 K, sky share +0.5 pp, sky a few % brighter. Not needed for the colour; AOD 0.1 and Angstrom 1.3 stay
   (inside the AERONET climatology, global CCT insensitive). No ozone or solar-spectrum change.

## Open points
- The literature ranges are quoted as given (brief, t5b: Hernandez-Andres et al. 2001; Judd et al. 1964); SSA / AOD
  climatology numbers are from memory and should be checked against the papers before a config change.
- SPCTRAL2 is only the sky's proxy for differences; the real sky is untouched by proposal 1, so the 5700 K estimate uses
  the core's own sky (probe numbers of t5b fix 2) plus the exact direct beam.
