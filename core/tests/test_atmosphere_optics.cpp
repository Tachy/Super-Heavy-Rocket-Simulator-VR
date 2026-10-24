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
    // Bucholtz 1995 Table 3 (US standard atmosphere, surface scattering coefficient): 0.55 um -> 1.15e-5 /m.
    CHECK(environment::rayleigh_cross_section_m2(550e-9) * n_s == Approx(1.15e-5).epsilon(0.01));
    // 400 nm and 700 nm: the table rows are reproduced by the published formula (Peck & Reeder refractivity, King
    // factor 1.0616 / 1.0488 from the depolarisation ratios 2.955 % / 2.81 %), evaluated independently in Python:
    // 4.2609e-5 /m and 4.3117e-6 /m.
    CHECK(environment::rayleigh_cross_section_m2(400e-9) * n_s == Approx(4.2609e-5).epsilon(0.01));
    CHECK(environment::rayleigh_cross_section_m2(700e-9) * n_s == Approx(4.3117e-6).epsilon(0.01));
    // Independent physical check: sigma ~ lambda^-4.08 between 400 and 700 nm (Bucholtz 1995, eq. 12 fit).
    const double ratio = environment::rayleigh_cross_section_m2(400e-9) / environment::rayleigh_cross_section_m2(700e-9);
    CHECK(std::log(ratio) / std::log(700.0 / 400.0) == Approx(4.08).epsilon(0.02));
    // Peck & Reeder 1972: standard air n - 1 = 2.7782e-4 at 550 nm (about 2.93e-4 at 0.5 um sodium D 589 nm: 2.7728e-4).
    CHECK(environment::air_refractivity(589.29e-9) == Approx(2.7728e-4).epsilon(0.002));
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
    // Chappuis band maximum near 600 nm: Bruneton / Serdyuchenko bin 600-610 nm ~ 5e-25 m^2.
    CHECK(environment::ozone_cross_section_m2(605e-9) == Approx(5.0e-25).epsilon(0.05));
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

TEST_CASE("Solar spectrum integrates to the visible E-490 band", "[atmosphere]") {
    // ASTM E-490 AM0 (1366.1 W/m^2 total): 400 - 700 nm carries about 531 W/m^2 (38.9 %).
    const double visible = simpson([](double l) { return environment::solar_irradiance_w_m2_per_m(l); }, 400e-9, 700e-9, 600);
    CHECK(visible == Approx(531.0).epsilon(0.01));
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
    CHECK(optics.scattering_orders == 4);

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
