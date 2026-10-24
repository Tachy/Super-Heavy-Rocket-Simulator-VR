#pragma once

#include "starsim/config/atmosphere_optics_config.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::environment {

// Spectral optical inputs of the Earth atmosphere for the precomputed scattering (M8 atmosphere). SI units; the
// wavelength is in metres. Outside the tabulated range the spectra clamp to their end values.

// Refractivity n - 1 of standard air (15 C, 101325 Pa, 300 ppm CO2) [S: Peck & Reeder 1972, JOSA 62, 958].
double air_refractivity(double wavelength_m);

// Rayleigh scattering cross section per molecule of air [S: Bucholtz 1995, Appl. Opt. 34, 2765]: the Rayleigh formula
// with the King correction factor of the wavelength-dependent depolarisation.
double rayleigh_cross_section_m2(double wavelength_m);

// Molecule number density of air p / (k T) of the US76 atmosphere; 0 above 1000 km (and below -5 km clamped like US76).
double air_number_density_per_m3(double altitude_m);

// Ozone number density: the AFGL US Standard profile [S: Anderson et al. 1986, AFGL-TR-86-0110, Table 1f], linear
// between its rows, 0 above 120 km, scaled to the column in Dobson units. The column is normalised from 0 m: rays from
// the model ground below the ellipsoid (-150 m) see a negligible extra amount (150 m of surface-level ozone, far below 0.1 % of the column).
double ozone_number_density_per_m3(double altitude_m, double column_du);

// Ozone absorption cross section [S: Serdyuchenko 2014 via Bruneton 2017], 10 nm bins, linear.
double ozone_cross_section_m2(double wavelength_m);

// Aerosol extinction coefficient: exponential profile of the configured scale height, normalised so that the
// vertical column from 0 m to the model top has the configured optical depth at 550 nm; Angstrom spectral law.
// The normalisation starts at 0 m (ruling, kept): the profile continues below, so the column from the model ground
// at -150 m carries about 13 % more optical depth (exp(150 / 1200) - 1) than the configured value.
double aerosol_extinction_per_m(double wavelength_m, double altitude_m, const config::AtmosphereOpticsConfig& optics);

// Extraterrestrial solar spectral irradiance at 1 AU [S: ASTM G-173-03 ETR via Bruneton, 10 nm bins], per metre of
// wavelength (W m^-2 m^-1).
double solar_irradiance_w_m2_per_m(double wavelength_m);

// CIE 1931 2-degree colour matching functions xbar, ybar, zbar (linear interpolation of the 5 nm table).
math::Vec3 cie_1931_cmf(double wavelength_m);

}  // namespace starsim::environment
