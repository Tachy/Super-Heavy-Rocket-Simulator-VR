#include "starsim/environment/atmosphere_optics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>
#include <vector>

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

// Depolarisation ratio of air in percent [S: Bucholtz 1995, Appl. Opt. 34, 2765, Table 1, column rho_n; read from the
// published table at https://opg.optica.org/abstract.cfm?URI=ao-34-15-2765]: {wavelength um, rho in percent}; linear between
// rows, constant beyond. The King factor is computed from rho (Table 1 lists it rounded to 4 digits).
constexpr std::array<std::pair<double, double>, 36> depolarisation_percent = {{
    {0.200, 4.545}, {0.205, 4.384}, {0.210, 4.221}, {0.215, 4.113}, {0.220, 4.004}, {0.225, 3.895},
    {0.230, 3.785}, {0.240, 3.675}, {0.250, 3.565}, {0.260, 3.455}, {0.270, 3.400}, {0.280, 3.289},
    {0.290, 3.233}, {0.300, 3.178}, {0.310, 3.178}, {0.320, 3.122}, {0.330, 3.066}, {0.340, 3.066},
    {0.350, 3.010}, {0.360, 3.010}, {0.370, 3.010}, {0.380, 2.955}, {0.390, 2.955}, {0.400, 2.955},
    {0.450, 2.899}, {0.500, 2.842}, {0.550, 2.842}, {0.600, 2.786}, {0.650, 2.786}, {0.700, 2.786},
    {0.750, 2.786}, {0.800, 2.730}, {0.850, 2.730}, {0.900, 2.730}, {0.950, 2.730}, {1.000, 2.730},
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

// US Standard Atmosphere ozone profile [S: Anderson, Clough, Kneizys, Chetwynd & Shettle 1986, "AFGL Atmospheric
// Constituent Profiles (0-120 km)", AFGL-TR-86-0110, Table 1f (U.S. Standard, derived from the 1976 US Standard
// Atmosphere), https://archive.org/details/DTIC_ADA175173; the numbers were read from the machine-readable copy
// table_1f.csv of the joseki package, https://github.com/rayference/joseki]. Rows: {altitude km, air number density
// cm^-3, ozone volume mixing ratio ppmv}. The ozone number density is the product, normalised to the column by the
// caller (the table's own column is about 345 DU).
struct OzoneRow {
    double altitude_km;
    double air_per_cm3;
    double ozone_ppmv;
};
constexpr OzoneRow afgl_us_standard_ozone[] = {
    {0, 2.548e19, 2.66e-2}, {1, 2.313e19, 2.93e-2}, {2, 2.094e19, 3.24e-2}, {3, 1.891e19, 3.32e-2},
    {4, 1.704e19, 3.39e-2}, {5, 1.532e19, 3.77e-2}, {6, 1.373e19, 4.11e-2}, {7, 1.228e19, 5.01e-2},
    {8, 1.094e19, 5.97e-2}, {9, 9.719e18, 9.17e-2}, {10, 8.602e18, 1.31e-1}, {11, 7.589e18, 2.15e-1},
    {12, 6.489e18, 3.10e-1}, {13, 5.546e18, 3.85e-1}, {14, 4.739e18, 5.03e-1}, {15, 4.050e18, 6.51e-1},
    {16, 3.462e18, 8.70e-1}, {17, 2.960e18, 1.19}, {18, 2.530e18, 1.59}, {19, 2.163e18, 2.03},
    {20, 1.849e18, 2.58}, {21, 1.575e18, 3.03}, {22, 1.342e18, 3.65}, {23, 1.144e18, 4.17},
    {24, 9.765e17, 4.63}, {25, 8.337e17, 5.12}, {27.5, 5.640e17, 5.80}, {30, 3.830e17, 6.55},
    {32.5, 2.524e17, 7.37}, {35, 1.761e17, 7.84}, {37.5, 1.238e17, 7.80}, {40, 8.310e16, 7.30},
    {42.5, 5.803e16, 6.20}, {45, 4.090e16, 5.25}, {47.5, 2.920e16, 4.10}, {50, 2.136e16, 3.10},
    {55, 1.181e16, 1.80}, {60, 6.426e15, 1.10}, {65, 3.386e15, 0.70}, {70, 1.723e15, 0.30},
    {75, 8.347e14, 0.25}, {80, 3.832e14, 0.30}, {85, 1.711e14, 0.50}, {90, 7.136e13, 0.70},
    {95, 2.924e13, 0.70}, {100, 1.189e13, 0.40}, {105, 5.033e12, 0.20}, {110, 2.144e12, 0.05},
    {115, 9.688e11, 5.0e-3}, {120, 5.114e11, 5.0e-4},
};

// {altitude km, relative ozone number density (air x mixing ratio)}.
const std::vector<std::pair<double, double>> ozone_profile = [] {
    std::vector<std::pair<double, double>> profile;
    for (const OzoneRow& r : afgl_us_standard_ozone) profile.emplace_back(r.altitude_km, r.air_per_cm3 * r.ozone_ppmv);
    return profile;
}();

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
