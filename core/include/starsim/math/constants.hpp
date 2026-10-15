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

// Mean synodic month (new moon to new moon) [S: Meeus, Astronomical Algorithms, 2nd ed., ch. 49: 29.530588861 d].
inline constexpr double synodic_month_s = 29.530588861 * 86400.0;

// Mean sidereal month [S: Meeus, Astronomical Algorithms, 2nd ed., ch. 49: 27.321661 d].
inline constexpr double sidereal_month_s = 27.321661 * 86400.0;

// IAU 2015 Resolution B3 nominal solar radius.
inline constexpr double sun_radius_m = 695700000.0;

// Mean obliquity of the ecliptic at J2000.0 (IAU 2006): 84381.406 arcsec.
inline constexpr double obliquity_j2000_rad = 84381.406 / 3600.0 * deg_to_rad;

}  // namespace starsim::constants
