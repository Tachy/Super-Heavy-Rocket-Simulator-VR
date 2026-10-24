#include "starsim/environment/atmosphere_optics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>

#include "starsim/environment/atmosphere.hpp"
#include "starsim/environment/atmosphere_spectra.hpp"
#include "starsim/math/constants.hpp"

namespace starsim::environment {

namespace {

constexpr double nm = 1e-9;

// Linear interpolation of a table of values at the centres of the 10 nm bins, clamped at the ends. The tabulated
// value is the bin average, so it belongs at the bin centre.
double bin_value(const double (&table)[spectra::bin_count], double wavelength_m) {
    const double position = (wavelength_m / nm - spectra::bin_first_wavelength_nm) / spectra::bin_width_nm - 0.5;
    if (position <= 0.0) return table[0];
    if (position >= spectra::bin_count - 1) return table[spectra::bin_count - 1];
    const int i = static_cast<int>(position);
    const double u = position - i;
    return table[i] * (1.0 - u) + table[i + 1] * u;
}

// Depolarisation ratio of air in percent [S: Bucholtz 1995, Table 1 (values of Penndorf 1957 / Young 1980,
// 0.2 - 1.0 um)]: {wavelength um, rho %}; linear between rows, constant beyond.
constexpr std::array<std::pair<double, double>, 29> depolarisation_percent = {{
    {0.200, 4.545}, {0.205, 4.384}, {0.210, 4.221}, {0.215, 4.113}, {0.220, 4.004}, {0.225, 3.895},
    {0.230, 3.785}, {0.240, 3.675}, {0.250, 3.565}, {0.260, 3.455}, {0.270, 3.400}, {0.280, 3.289},
    {0.290, 3.233}, {0.300, 3.178}, {0.310, 3.178}, {0.320, 3.122}, {0.330, 3.066}, {0.340, 3.066},
    {0.350, 3.010}, {0.370, 3.010}, {0.400, 2.955}, {0.450, 2.899}, {0.500, 2.843}, {0.550, 2.843},
    {0.600, 2.843}, {0.750, 2.787}, {0.800, 2.787}, {1.000, 2.730}, {1.500, 2.730},
}};

double king_factor(double wavelength_um) {
    const auto& t = depolarisation_percent;
    double rho_percent = t.front().second;
    if (wavelength_um >= t.back().first) {
        rho_percent = t.back().second;
    } else if (wavelength_um > t.front().first) {
        const auto upper = std::upper_bound(t.begin(), t.end(), wavelength_um,
                                            [](double w, const auto& row) { return w < row.first; });
        const auto lower = upper - 1;
        const double u = (wavelength_um - lower->first) / (upper->first - lower->first);
        rho_percent = lower->second * (1.0 - u) + upper->second * u;
    }
    const double rho = rho_percent * 0.01;
    return (6.0 + 3.0 * rho) / (6.0 - 7.0 * rho);
}

// Ozone number density shape [E: mid-latitude profile after the US Standard Atmosphere 1976 / Krueger & Minzner
// 1976 ozone model, rounded]: {altitude km, relative density}; normalised to the column by the caller.
constexpr std::array<std::pair<double, double>, 28> ozone_profile = {{
    {0, 0.40},  {2, 0.45},  {4, 0.50},  {6, 0.60},   {8, 0.75},   {10, 1.00}, {12, 1.50}, {14, 2.20},
    {16, 3.20}, {18, 4.30}, {20, 5.00}, {22, 5.20},  {24, 5.00},  {26, 4.40}, {28, 3.40}, {30, 2.50},
    {32, 1.75}, {34, 1.20}, {36, 0.80}, {38, 0.55},  {40, 0.38},  {45, 0.17}, {50, 0.08}, {60, 0.03},
    {70, 0.012}, {80, 0.005}, {90, 0.003}, {100, 0.0},
}};

double ozone_shape_integral_m() {
    static const double integral = [] {
        double sum = 0.0;
        for (size_t i = 1; i < ozone_profile.size(); ++i) {
            sum += 0.5 * (ozone_profile[i].second + ozone_profile[i - 1].second) *
                   (ozone_profile[i].first - ozone_profile[i - 1].first) * 1000.0;
        }
        return sum;
    }();
    return integral;
}

double ozone_shape(double altitude_m) {
    const double km = altitude_m / 1000.0;
    if (km <= ozone_profile.front().first) return ozone_profile.front().second;
    if (km >= ozone_profile.back().first) return 0.0;
    const auto upper = std::upper_bound(ozone_profile.begin(), ozone_profile.end(), km,
                                        [](double h, const auto& row) { return h < row.first; });
    const auto lower = upper - 1;
    const double u = (km - lower->first) / (upper->first - lower->first);
    return lower->second * (1.0 - u) + upper->second * u;
}

}  // namespace

double air_refractivity(double wavelength_m) {
    const double sigma_um = 1e-6 / wavelength_m;  // wavenumber in 1/um
    const double s2 = sigma_um * sigma_um;
    return 1e-8 * (8060.51 + 2480990.0 / (132.274 - s2) + 17455.7 / (39.32957 - s2));
}

double rayleigh_cross_section_m2(double wavelength_m) {
    const double n_minus_1 = air_refractivity(wavelength_m);
    const double n2_minus_1 = 2.0 * n_minus_1 + n_minus_1 * n_minus_1;
    const double n2_plus_2 = n2_minus_1 + 3.0;
    const double n_air = constants::standard_air_number_density_per_m3;
    const double l2 = wavelength_m * wavelength_m;
    const double pi3 = constants::pi * constants::pi * constants::pi;
    return 24.0 * pi3 * n2_minus_1 * n2_minus_1 / (l2 * l2 * n_air * n_air * n2_plus_2 * n2_plus_2) *
           king_factor(wavelength_m * 1e6);
}

double air_number_density_per_m3(double altitude_m) {
    if (altitude_m >= 1000e3) return 0.0;
    static const Us76Atmosphere us76;
    const AtmosphereState s = us76.at(altitude_m);
    return s.pressure_pa / (constants::boltzmann_j_per_k * s.temperature_k);
}

double ozone_number_density_per_m3(double altitude_m, double column_du) {
    return column_du * constants::dobson_unit_per_m2 * ozone_shape(altitude_m) / ozone_shape_integral_m();
}

double ozone_cross_section_m2(double wavelength_m) { return bin_value(spectra::ozone_cross_section_m2, wavelength_m); }

double aerosol_extinction_per_m(double wavelength_m, double altitude_m, const config::AtmosphereOpticsConfig& optics) {
    const double h = optics.aerosol_scale_height_m;
    const double column_m = h * (1.0 - std::exp(-optics.top_altitude_m / h));  // integral of exp(-z/h), 0..top
    const double spectral = std::pow(wavelength_m / (550.0 * nm), -optics.aerosol_angstrom_exponent);
    return optics.aerosol_optical_depth_550nm / column_m * std::exp(-altitude_m / h) * spectral;
}

double solar_irradiance_w_m2_per_m(double wavelength_m) {
    return bin_value(spectra::solar_irradiance_w_m2_nm, wavelength_m) * 1e9;  // per nm -> per m
}

math::Vec3 cie_1931_cmf(double wavelength_m) {
    const auto& t = spectra::cie_1931_2deg_cmf;
    const double position =
        std::clamp((wavelength_m / nm - t[0][0]) / 5.0, 0.0, static_cast<double>(spectra::cmf_rows - 1));
    const int i = std::min(static_cast<int>(position), spectra::cmf_rows - 2);
    const double u = position - i;
    const auto lerp = [&](int c) { return t[i][c] * (1.0 - u) + t[i + 1][c] * u; };
    return {lerp(1), lerp(2), lerp(3)};
}

}  // namespace starsim::environment
