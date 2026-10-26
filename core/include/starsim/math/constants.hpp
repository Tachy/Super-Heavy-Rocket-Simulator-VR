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
// Lifting condensation level, exact form (Romps 2017, "Exact expression for the lifting condensation level", J. Atmos.
// Sci. 74, 3891-3900, doi:10.1175/JAS-D-17-0102.1; constants as in the paper's published code, a constant-heat-capacity
// ideal-gas model of vapour and liquid whose saturation pressure follows Kirchhoff's law).
inline constexpr double romps_triple_point_temperature_k = 273.16;      // [S: Romps 2017]
inline constexpr double romps_triple_point_pressure_pa = 611.65;        // [S: Romps 2017]
inline constexpr double romps_vapour_internal_energy_at_triple_j_kg = 2.3740e6;  // [S: Romps 2017, E_0v]
inline constexpr double romps_gravity_mps2 = 9.81;                      // [S: Romps 2017, g]
inline constexpr double romps_dry_air_gas_constant_j_kgk = 287.04;      // [S: Romps 2017, R_a]
inline constexpr double romps_vapour_gas_constant_j_kgk = 461.0;        // [S: Romps 2017, R_v]
inline constexpr double romps_dry_air_cv_j_kgk = 719.0;                 // [S: Romps 2017, c_va]
inline constexpr double romps_vapour_cv_j_kgk = 1418.0;                 // [S: Romps 2017, c_vv]
inline constexpr double romps_liquid_c_j_kgk = 4119.0;                  // [S: Romps 2017, c_vl]
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

// Cloud optics (M8 weather and clouds).
// Density of liquid water [S: 1000 kg/m^3 (999.84 at 0 deg C, 999.97 at 4 deg C), CRC Handbook of Chemistry and
// Physics]: the droplet material density of the extinction 3 LWC / (2 rho r_eff).
inline constexpr double liquid_water_density_kg_m3 = 1000.0;
// Density of ice Ih at 0 deg C [S: 917 kg/m^3, Petrenko & Whitworth 1999, Physics of Ice, Oxford, table 11.1].
inline constexpr double ice_density_kg_m3 = 917.0;
// Effective radius of cloud droplets [E: 10 um; low-level stratiform clouds measure ~5-13 um, marine ~9-10 um, Miles,
// Verlinde & Clothiaux 2000, J. Atmos. Sci. 57, 295; the usual single value of radiation schemes, Stephens 1978,
// J. Atmos. Sci. 35, 2123]. It also sets the droplet diameter (20 um) of the phase function fit.
inline constexpr double cloud_droplet_effective_radius_m = 10e-6;
// Effective radius of cloud ice particles [E: 30 um, a typical mid-latitude cirrus value (~25-30 um at ice water
// contents of 1e-6..1e-5 kg/m^3), Stephens, Tsay, Stackhouse & Flatau 1990, J. Atmos. Sci. 47, 1742; Fu 1996,
// J. Climate 9, 2058 for the 3 IWC / (2 rho_ice r_eff) form of the visible extinction].
inline constexpr double cloud_ice_effective_radius_m = 30e-6;
// Widths of the edge zone of a cumulus, over which the liquid water falls from the cloud's interior value to zero by
// entrainment of dry air: the zone varies along a cloud's perimeter (cloud_noise.hpp: the steep edge ramp, its width
// log-uniform between the two values by a width noise). [E: 20 m and 40 m, the tens of metres of the turbulent inner
// shell that aircraft cross at shallow-cumulus edges, Katzwinkel, Siebert, Heus & Shaw 2014, J. Atmos. Sci. 71, 2810
// (subsiding shells, Heus & Jonker 2008, J. Atmos. Sci. 65, 1003): a sunlit cumulus has a crisp rim; the ~200 m
// transition zone of the margin (Wang, Geerts & French 2009, J. Atmos. Sci. 66, 3660) is, by estimate, the cloud-edge
// humidity gradient, not the optical edge [E]. The log-uniform spread between the ends is an estimate.]
inline constexpr double cloud_edge_width_min_m = 20.0;
inline constexpr double cloud_edge_width_max_m = 40.0;
// Area-perimeter fractal dimension of cloud and rain areas [S: D = 1.35 +- 0.05 over areas of 1 to 1.2e6 km^2 (radar
// and GOES infrared), Lovejoy 1982, Science 216, 185]. The sub-grid cloud shapes continue it below the grid
// (cloud_noise.hpp: the fractal shape value; the level sets of a fractional Brownian field of Hurst exponent H have
// dimension 2 - H, Mandelbrot 1982, The Fractal Geometry of Nature, ch. 28).
inline constexpr double cloud_perimeter_fractal_dimension = 1.35;
// Thickness of the transition at a cloud base [E: 50 m, an estimate; the base of a convective cloud is the lifting
// condensation level of the rising air, flat and sharp over the cloud (Wallace & Hobbs 2006, Atmospheric Science, 2nd
// ed., Academic Press, sect. 3.5)] (cloud_render_texels.hpp: the vertical profile rule).
inline constexpr double cloud_base_transition_m = 50.0;
// A cloud fraction below this counts as none: half a step of the 8-bit cloud-fraction code (the same constant as
// CLOUD_MIN_FRACTION in shaders/clouds.gdshaderinc; the shadow lookup and the column cover use it too).
inline constexpr double cloud_min_cover = 0.5 / 255.0;
// Cover of the direct Sun's beam through a broken layer of cloud fraction f: 1 - (1 - f)^n, n = 1 + a (1 - f) + b max(cot
// e - k, 0) for the Sun at elevation e (cloud_optics.hpp: cloud_beam_cover). The shipped shapes' 3D noise does not
// overlap maximally: the clouds of the layers' heights are displaced against each other (a: the vertical part, the
// shadow's cover above f at the zenith), and a slanted beam crosses further, independent cells (b, k: its horizontal
// run through the layer, ~cot e times the layer's depth, beyond the cells' correlation length). [E] Fitted to the
// shapes' shadows (cloud_check.gd section 7, area means over eight independent sites, f 0.15-0.6, e 5-75 deg, a layer
// of 1 km): within 0.02 of the transmittance. The cloud-fraction 1 (decks) is untouched.
inline constexpr double cloud_overlap_vertical = 0.66;
inline constexpr double cloud_overlap_slant = 0.21;
inline constexpr double cloud_overlap_slant_onset_cot = 1.8;

}  // namespace starsim::constants
