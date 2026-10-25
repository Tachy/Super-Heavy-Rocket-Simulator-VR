#pragma once

// Physical and geodetic constants with their sources.
// Gravitational parameters (GM) are NOT defined here: they are read from the DE440-consistent
// kernel gm_de440.tpc at runtime (see ephemeris::Ephemeris::gm_m3ps2).

namespace starsim::constants {

inline constexpr double pi = 3.14159265358979323846;
inline constexpr double two_pi = 2.0 * pi;
inline constexpr double deg_to_rad = pi / 180.0;
inline constexpr double rad_to_deg = 180.0 / pi;

// Standard gravity, exact by definition (3rd CGPM, 1901).
inline constexpr double standard_gravity_mps2 = 9.80665;

// Sutton-Graves stagnation-point heating constant for Earth air, SI units (kg^0.5 m^-1; q in W/m^2 with rho in
// kg/m^3, nose radius in m, speed in m/s). Sutton & Graves, NASA TR R-376 (1971): 1.7415e-4.
inline constexpr double sutton_graves_earth_si = 1.7415e-4;

// Standard atmosphere sea-level pressure, exact by definition.
inline constexpr double standard_pressure_pa = 101325.0;
// US Standard Atmosphere 1976 sea-level density (NOAA-S/T 76-1562, Table I): the reference of equivalent airspeed.
inline constexpr double standard_sea_level_density_kg_m3 = 1.225;

// Seconds per Julian day; Julian date of the J2000.0 epoch (2000-01-01 12:00:00 TT).
inline constexpr double seconds_per_day = 86400.0;
inline constexpr double j2000_julian_date = 2451545.0;

// WGS84 ellipsoid (NIMA TR8350.2, 3rd edition).
inline constexpr double wgs84_semi_major_axis_m = 6378137.0;
inline constexpr double wgs84_flattening = 1.0 / 298.257223563;

// Earth gravitational parameter (IERS Conventions 2010, TT-compatible) and nominal rotation rate (WGS84,
// NIMA TR8350.2). For onboard models that must not call SPICE (entry guidance prediction).
inline constexpr double earth_gm_m3ps2 = 3.986004418e14;
inline constexpr double earth_rotation_rate_radps = 7.292115e-5;

// Earth second zonal harmonic J2 (unnormalised), EGM2008: C20 = -4.84165143790815e-4 normalised.
inline constexpr double earth_j2 = 1.0826267e-3;

// IAU 2015 nominal lunar mean radius used for the Moon sphere.
inline constexpr double moon_mean_radius_m = 1737400.0;
// Highest lunar terrain above the 1737.4 km sphere [S: Smith et al. 2010, GRL 37, L18204 (LOLA): 10,786 m].
inline constexpr double moon_max_terrain_height_m = 10786.0;

// Lunar gravitational parameter [S] DE440 (gm_de440.tpc, BODY301_GM = 4902.800118457549 km^3/s^2). For code without
// an ephemeris at hand; guidance and planning take the GM from the ReferenceBody / ephemeris.
inline constexpr double moon_gm_m3ps2 = 4.902800118457549e12;

// Mean synodic month (new moon to new moon) [S: Meeus, Astronomical Algorithms, 2nd ed., ch. 49: 29.530588861 d].
inline constexpr double synodic_month_s = 29.530588861 * 86400.0;

// Mean sidereal month [S: Meeus, Astronomical Algorithms, 2nd ed., ch. 49: 27.321661 d].
inline constexpr double sidereal_month_s = 27.321661 * 86400.0;

// IAU 2015 Resolution B3 nominal solar radius.
inline constexpr double sun_radius_m = 695700000.0;

// Mean obliquity of the ecliptic at J2000.0 (IAU 2006): 84381.406 arcsec.
inline constexpr double obliquity_j2000_rad = 84381.406 / 3600.0 * deg_to_rad;

// Thermal radiation (refuelling r2, boil-off).
// Total solar irradiance at 1 AU [S: Kopp & Lean 2011, GRL 38, L01706: 1360.8 +- 0.5 W/m^2].
inline constexpr double solar_constant_w_m2 = 1361.0;
// [S: IAU 2012 Resolution B2]
inline constexpr double astronomical_unit_m = 1.495978707e11;
// [S: SI definition of the metre]
inline constexpr double speed_of_light_mps = 299792458.0;
// [S: CODATA 2018]
inline constexpr double stefan_boltzmann_w_m2k4 = 5.670374419e-8;
// Earth's Bond albedo [S: 0.29-0.30, CERES; Stephens et al. 2015, Rev. Geophys. 53] - 0.30 as in the r2 spec.
inline constexpr double earth_bond_albedo = 0.30;
// Global mean outgoing longwave radiation [S: 239 W/m^2, Trenberth, Fasullo & Kiehl 2009, BAMS 90].
inline constexpr double earth_outgoing_longwave_w_m2 = 239.0;
// The Moon's bolometric Bond albedo [S: ~0.11 (0.07-0.16 maria / highlands), Vasavada et al. 2012, JGR 117 E00H18,
// LRO Diviner] (refuelling r5).
inline constexpr double moon_bond_albedo = 0.11;
// Equatorial night-time surface temperature before dawn [S: ~95 K, Vasavada et al. 2012, Diviner].
inline constexpr double moon_night_temperature_k = 95.0;
// Mean radius [S: IUGG 6371.0 km]: the spherical Earth of shadows and thermal radiation.
inline constexpr double earth_mean_radius_m = 6371000.0;

// JONSWAP spectrum shape [S: gamma 3.3 peak enhancement, sigma 0.07 below / 0.09 above the peak; Hasselmann et al. 1973,
// DNV-RP-C205 sec. 3.5.5] (sea state of the drone ship).
inline constexpr double jonswap_gamma = 3.3;
inline constexpr double jonswap_sigma_low = 0.07;
inline constexpr double jonswap_sigma_high = 0.09;

// Atmospheric optics (M8 atmosphere).
// [S: SI 2019 definition]
inline constexpr double boltzmann_j_per_k = 1.380649e-23;
// One Dobson unit as a column of molecules [S: 2.6867e20 m^-2, 0.01 mm of gas at 273.15 K and 101325 Pa;
// Loschmidt constant 2.6867811e25 m^-3 x 1e-5 m].
inline constexpr double dobson_unit_per_m2 = 2.6867e20;
// Refractivity n - 1 of dry standard air (288.15 K, 101325 Pa, density 1.2250 kg/m^3) at 550 nm, the centre of the
// visual band [S: Edlén 1966, Metrologia 2, 71, eq. 1: (n - 1) 1e8 = 8342.13 + 2406030 / (130 - s^2)
// + 15997 / (38.9 - s^2), s = 1/0.55 um^-1]. Refraction scales it with the density (Gladstone-Dale).
inline constexpr double standard_air_refractivity_550nm = 2.778244e-4;
// Molecule number density of standard air (288.15 K, 101325 Pa) used by the Rayleigh formula
// [S: Bucholtz 1995, Appl. Opt. 34, 2765, N_s = 2.54743e19 cm^-3].
inline constexpr double standard_air_number_density_per_m3 = 2.54743e25;
// Luminous efficacy of monochromatic 540 THz radiation, the photopic maximum K_m [S: SI 2019 definition of the
// candela, K_cd = 683 lm/W; Bruneton 2017 MAX_LUMINOUS_EFFICACY].
inline constexpr double max_luminous_efficacy_lm_per_w = 683.0;

// Weather snapshots (M8 weather and clouds).
// Specific gas constant of dry air [S: 287.05 J/(kg K), WMO / ECMWF IFS documentation Part IV (R_dry = 287.0597)].
inline constexpr double dry_air_gas_constant_j_kgk = 287.05;
// Specific gas constant of water vapour [S: 461.51 J/(kg K), Wallace & Hobbs 2006, Atmospheric Science, 2nd ed.,
// sec. 3.1]; the virtual temperature is T (1 + (R_v / R_d - 1) q).
inline constexpr double water_vapour_gas_constant_j_kgk = 461.51;
// Tropospheric lapse rate of the standard atmosphere [S: 6.5 K/km, US Standard Atmosphere 1976, NOAA-S/T 76-1562]:
// the temperature from the surface to the first pressure level above it.
inline constexpr double standard_lapse_rate_k_per_m = 0.0065;
// Scale height of the water-vapour density [E: ~2 km, the usual tropospheric value, e.g. Weaver & Ramanathan 1995,
// JGR 100(D6), 11585]: spreads ERA5's total column water vapour over the pressure levels (virtual temperature);
// 1.5 / 2 / 2.5 km give 8.5 / 9.5 / 11.3 m max height error over the six fixture columns.
inline constexpr double water_vapour_scale_height_m = 2000.0;
// WGS84 normal gravity (NIMA TR8350.2, 3rd edition, sec. 4.2): equatorial gravity, Somigliana constant k, first
// eccentricity squared, and m = omega^2 a^2 b / GM of the free-air gradient (eq. 4-3).
inline constexpr double wgs84_equatorial_gravity_mps2 = 9.7803253359;
inline constexpr double wgs84_somigliana_k = 0.00193185265241;
inline constexpr double wgs84_first_eccentricity_squared = 0.00669437999013;
inline constexpr double wgs84_gravity_ratio_m = 0.00344978600308;

}  // namespace starsim::constants
