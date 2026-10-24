// Spectral optical inputs of the Earth atmosphere (M8 atmosphere, Task 1): Rayleigh, ozone, aerosol, solar spectrum
// and colour matching functions against published values.

#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/config/atmosphere_optics_config.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/environment/atmosphere_optics.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using Catch::Approx;

namespace {

const std::filesystem::path optics_toml = std::filesystem::path(STARSIM_CONFIG_DIR) / "atmosphere" / "earth_optics.toml";

// Simpson integral of f over [a, b] with n (even) intervals.
template <class F>
double simpson(F f, double a, double b, int n) {
    const double h = (b - a) / n;
    double sum = f(a) + f(b);
    for (int i = 1; i < n; ++i) sum += f(a + i * h) * (i % 2 ? 4.0 : 2.0);
    return sum * h / 3.0;
}

}  // namespace

TEST_CASE("Rayleigh scattering matches Bucholtz 1995", "[atmosphere]") {
    const double n_s = constants::standard_air_number_density_per_m3;  // 2.54743e25 /m^3
    // Bucholtz 1995, Appl. Opt. 34, 2765, Table 2 (standard air, 1013.25 mbar, 288.15 K; read at
    // https://opg.optica.org/abstract.cfm?URI=ao-34-15-2765): cross section sigma in cm^2 and volume-scattering
    // coefficient beta_s in km^-1; beta_s = sigma N_s.
    //   0.40 um: sigma 1.673e-26 cm^2, beta_s 4.261e-2 /km    0.55 um: 4.509e-27 cm^2, 1.149e-2 /km
    //   0.70 um: sigma 1.692e-27 cm^2, beta_s 4.310e-3 /km
    CHECK(environment::rayleigh_cross_section_m2(400e-9) == Approx(1.673e-30).epsilon(0.005));
    CHECK(environment::rayleigh_cross_section_m2(550e-9) == Approx(4.509e-31).epsilon(0.005));
    CHECK(environment::rayleigh_cross_section_m2(700e-9) == Approx(1.692e-31).epsilon(0.005));
    CHECK(environment::rayleigh_cross_section_m2(400e-9) * n_s == Approx(4.261e-5).epsilon(0.005));
    CHECK(environment::rayleigh_cross_section_m2(550e-9) * n_s == Approx(1.149e-5).epsilon(0.005));
    CHECK(environment::rayleigh_cross_section_m2(700e-9) * n_s == Approx(4.310e-6).epsilon(0.005));
    // Independent physical check: sigma ~ lambda^-4.08 between 400 and 700 nm (Bucholtz 1995 power-law fit).
    const double ratio = environment::rayleigh_cross_section_m2(400e-9) / environment::rayleigh_cross_section_m2(700e-9);
    CHECK(std::log(ratio) / std::log(700.0 / 400.0) == Approx(4.08).epsilon(0.02));
    // Peck & Reeder 1972, standard air: n - 1 = 2.7782e-4 at 550 nm (evaluating their dispersion formula by hand;
    // the formula is the implementation, so this pins the arithmetic, not an independent table).
    CHECK(environment::air_refractivity(550e-9) == Approx(2.7782e-4).epsilon(0.001));
}

TEST_CASE("Vertical Rayleigh optical depth matches Bucholtz 1995", "[atmosphere]") {
    const double sigma = environment::rayleigh_cross_section_m2(550e-9);
    const double tau =
        sigma * simpson([](double h) { return environment::air_number_density_per_m3(h); }, 0.0, 100e3, 2000);
    // Bucholtz 1995, Table 3, US standard atmosphere, 0.55 um: 0.0973.
    CHECK(tau == Approx(0.0973).epsilon(0.02));
}

TEST_CASE("Ozone profile integrates to its column", "[atmosphere]") {
    const double column =
        simpson([](double h) { return environment::ozone_number_density_per_m3(h, 300.0); }, 0.0, 100e3, 20000);
    CHECK(column == Approx(300.0 * 2.6867e20).epsilon(0.005));
    CHECK(environment::ozone_number_density_per_m3(150e3, 300.0) == 0.0);
    // AFGL US Standard profile (Anderson et al. 1986, AFGL-TR-86-0110, Table 1f): at 22 km 1.342e18 cm^-3 air x
    // 3.65 ppmv = 4.898e18 m^-3; the table's own column is 345.8 DU (trapezoid), so scaled to 300 DU the density is
    // 4.898e18 x 300 / 345.8 = 4.25e18 m^-3.
    CHECK(environment::ozone_number_density_per_m3(22e3, 300.0) == Approx(4.249e18).epsilon(0.005));
    // Transcription check of Bruneton's table (Chappuis maximum, bin 600-610 nm = 5.019e-25 m^2 at its centre 605 nm).
    CHECK(environment::ozone_cross_section_m2(605e-9) == Approx(5.019e-25).epsilon(0.001));
}

TEST_CASE("Aerosol optical depth at 550 nm is the configured one", "[atmosphere]") {
    const auto optics = config::load_atmosphere_optics(optics_toml);
    const double tau = simpson([&](double h) { return environment::aerosol_extinction_per_m(550e-9, h, optics); }, 0.0,
                               optics.top_altitude_m, 20000);
    CHECK(tau == Approx(optics.aerosol_optical_depth_550nm).epsilon(0.005));
    // Angstrom law: 400 nm is (550/400)^alpha times stronger.
    const double ratio = environment::aerosol_extinction_per_m(400e-9, 0.0, optics) /
                         environment::aerosol_extinction_per_m(550e-9, 0.0, optics);
    CHECK(ratio == Approx(std::pow(550.0 / 400.0, optics.aerosol_angstrom_exponent)).epsilon(1e-9));
}

TEST_CASE("Solar spectrum integrates to the visible G-173 ETR band", "[atmosphere]") {
    // ASTM G-173-03 extraterrestrial (ETR) column: trapezoid integral 400 - 700 nm of the official data, computed from
    // the NREL astmg173.xls as shipped in pvlib 0.16.1 (pvlib/data/ASTMG173.csv, 0.5 / 1 nm grid) = 529.96 W/m^2
    // (whole file 280 - 4000 nm: 1347.9 W/m^2). Our 10 nm bin table (Bruneton) must reproduce it within 0.5 %.
    const double visible = simpson([](double l) { return environment::solar_irradiance_w_m2_per_m(l); }, 400e-9, 700e-9, 600);
    CHECK(visible == Approx(529.96).epsilon(0.005));
    // CIE 1931 2-degree: ybar(555 nm) = 1.
    CHECK(environment::cie_1931_cmf(555e-9).y == Approx(1.0).epsilon(0.002));
    // zbar peaks in the blue (445 nm ~ 1.78), xbar has its main peak near 600 nm (1.06).
    CHECK(environment::cie_1931_cmf(445e-9).z == Approx(1.7826).epsilon(0.02));
    CHECK(environment::cie_1931_cmf(600e-9).x == Approx(1.0622).epsilon(0.02));
}

TEST_CASE("Optics config loads and refuses missing keys", "[atmosphere]") {
    const auto optics = config::load_atmosphere_optics(optics_toml);
    CHECK(optics.top_altitude_m == 100e3);
    CHECK(optics.ground_altitude_m == -150.0);
    CHECK(optics.scattering_orders == 5);

    std::ifstream in(optics_toml);
    std::string line;
    std::ostringstream kept;
    while (std::getline(in, line)) {
        if (line.rfind("ground_albedo", 0) != 0) kept << line << '\n';
    }
    const auto broken = std::filesystem::temp_directory_path() / "starsim_atmosphere_optics_no_albedo.toml";
    {
        std::ofstream out(broken);
        out << kept.str();
    }
    CHECK_THROWS_AS(config::load_atmosphere_optics(broken), config::ConfigError);
    std::filesystem::remove(broken);
}
