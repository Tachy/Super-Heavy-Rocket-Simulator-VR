// Precomputed atmosphere tables (M8 atmosphere, Task 2): transmittance and single scattering of the Bruneton port
// against the independent brute-force integrator in atmosphere_reference.hpp.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <functional>
#include <memory>
#include <set>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "atmosphere_reference.hpp"
#include "starsim/config/atmosphere_optics_config.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/environment/atmosphere_scattering.hpp"
#include "starsim/environment/atmosphere_tables.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using Catch::Approx;
namespace ref = starsim::test::atmosphere_reference;

namespace {

const std::filesystem::path optics_toml = std::filesystem::path(STARSIM_CONFIG_DIR) / "atmosphere" / "earth_optics.toml";

config::AtmosphereOpticsConfig optics() { return config::load_atmosphere_optics(optics_toml); }

// Transmittance and single scattering only (orders = 1): the scattering table then holds the single Rayleigh
// scattering the port check compares.
environment::AtmosphereTables precompute_timed(config::AtmosphereOpticsConfig c,
                                               const environment::AtmosphereTableSizes& sizes = {}) {
    c.scattering_orders = 1;
    const auto t0 = std::chrono::steady_clock::now();
    auto tables = environment::precompute_atmosphere(c, sizes);
    const double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::printf("atmosphere precompute (transmittance + single scattering, %dx%dx%dx%d): %.2f s\n", sizes.scattering_nu,
                sizes.scattering_mu_s, sizes.scattering_mu, sizes.scattering_r, s);
    return tables;
}

double rel_error(double value, double reference) { return std::abs(value - reference) / std::abs(reference); }

}  // namespace

TEST_CASE("Transmittance table matches a direct optical-depth integral", "[atmosphere]") {
    const auto c = optics();
    const auto tables = precompute_timed(c);
    const auto& p = tables.parameters;
    const ref::Model model{c};
    const std::vector<double> lambdas{environment::spectra::lambda_r_nm * ref::nm, environment::spectra::lambda_g_nm * ref::nm,
                                      environment::spectra::lambda_b_nm * ref::nm};

    // Zenith, 60 deg, 85 deg, horizontal and the ray grazing the model ground (its horizon) from 0, 10 and 50 km.
    // Measured max 3.2e-3 (blue, grazing from 0 km; 1024 x 1024 table), Bruneton's 256 x 64 table: 0.82.
    double max_error = 0.0;
    for (const double altitude_m : {0.0, 10e3, 50e3}) {
        const double r = constants::wgs84_semi_major_axis_m + altitude_m;
        const double mu_horizon = -std::sqrt(1.0 - std::pow(p.bottom_radius_m / r, 2));
        for (const double mu : {1.0, std::cos(60.0 * constants::deg_to_rad), std::cos(85.0 * constants::deg_to_rad),
                                0.0, mu_horizon}) {
            const auto table = environment::lookup_transmittance_to_top(tables, r, mu);
            const auto expected = ref::transmittance_to_top(model, lambdas, r, mu);
            for (int ch = 0; ch < 3; ++ch) {
                const double e = rel_error(table[ch], expected[ch]);
                max_error = std::max(max_error, e);
                INFO("h " << altitude_m << " mu " << mu << " ch " << ch << " table " << table[ch] << " ref "
                          << expected[ch]);
                CHECK(e < 0.005);
            }
        }
    }
    std::printf("transmittance: max relative error %.3e\n", max_error);
}

// "Single scattering matches a brute-force raymarch" (the table texels against the reference) runs on the tables of the case
// "Single-scattering lookups between table rows and across the horizon match the raymarch" in test_atmosphere_scattering.cpp:
// the same orders = 1 precompute at the same default grid, computed once.

TEST_CASE("The ground radius lies below every relevant surface", "[atmosphere]") {
    const auto c = optics();
    const auto p = environment::atmosphere_parameters(c);
    const double a = constants::wgs84_semi_major_axis_m;
    // EGM2008 geoid minimum about -106 m (Indian Ocean geoid low; config/atmosphere/earth_optics.toml).
    CHECK(p.bottom_radius_m < a - 106.0);
    CHECK(p.bottom_radius_m == a + c.ground_altitude_m);
    CHECK(p.top_radius_m == a + c.top_altitude_m);
    for (const auto& site : config::load_launch_sites(STARSIM_CONFIG_DIR)) {
        INFO(site.id);
        CHECK(a + site.height_m > p.bottom_radius_m);
    }
}

TEST_CASE("Atmosphere parameters follow Bruneton's luminance mode", "[atmosphere]") {
    const auto p = environment::atmosphere_parameters(optics());
    // Bruneton's float-precision value (demo.cc): the Sun to 30 deg below the horizon, twilight to -18 deg inside.
    CHECK(p.mu_s_min == Approx(std::cos(120.0 * constants::deg_to_rad)));
    CHECK(p.sun_angular_radius_rad == Approx(std::asin(constants::sun_radius_m / constants::astronomical_unit_m)));
    for (int c = 0; c < 3; ++c) CHECK(p.sky_radiance_to_luminance[c] == constants::max_luminous_efficacy_lm_per_w);
    // Solar luminous constant 133.8 klx [S: Darula, Kittler & Gueymard 2005, Solar Energy 79, 559] for 1361 W/m^2;
    // the G-173 ETR spectrum carries ~1 % less (1347.9 W/m^2).
    CHECK(p.toa_solar_illuminance_lux > 128e3);
    CHECK(p.toa_solar_illuminance_lux < 136e3);
    // The Sun's luminance from the RGB factors: Y = 0.2126 R + 0.7152 G + 0.0722 B of E_rgb x k_sun ~ E_v.
    double y = 0.0;
    const double weights[3] = {0.2126, 0.7152, 0.0722};
    for (int c = 0; c < 3; ++c) y += weights[c] * p.solar_illuminance_rgb[c] * p.sun_radiance_to_luminance[c];
    CHECK(y == Approx(p.toa_solar_illuminance_lux).epsilon(0.02));
    // The game reads the tables' ground albedo from the parameters (sky ground, umbra light; final review M3).
    CHECK(p.ground_albedo == optics().ground_albedo);
}

TEST_CASE("Table parameterisation round-trips", "[atmosphere]") {
    const environment::AtmosphereTableSizes sizes;
    const auto p = environment::atmosphere_parameters(optics());
    for (const double alt : {-150.0, 0.0, 5e3, 60e3, 100e3}) {
        const double r = constants::wgs84_semi_major_axis_m + alt;
        for (const double mu : {1.0, 0.3, 0.01}) {
            const auto uv = environment::transmittance_uv_from_r_mu(p, sizes, r, mu);
            const auto back = environment::r_mu_from_transmittance_uv(p, sizes, uv);
            CHECK(back.r_m == Approx(r).epsilon(1e-9));
            if (alt < 100e3) CHECK(back.mu == Approx(mu).margin(1e-7));
        }
        for (const double mu : {0.8, -0.2, -0.9}) {
            const environment::ScatteringCell cell{r, mu, 0.4, 0.1, environment::ray_intersects_ground(p, r, mu)};
            const auto uvwz = environment::scattering_uvwz_from_cell(p, sizes, cell);
            const auto back = environment::scattering_cell_from_uvwz(p, sizes, uvwz);
            CHECK(back.r_m == Approx(r).epsilon(1e-9));
            if (alt > -150.0 && alt < 100e3) CHECK(back.mu == Approx(mu).margin(1e-7));
            CHECK(back.mu_s == Approx(0.4).margin(1e-9));
            CHECK(back.nu == Approx(0.1).margin(1e-12));
            CHECK(back.ray_r_mu_intersects_ground == cell.ray_r_mu_intersects_ground);
        }
    }
}

TEST_CASE("Input hash changes with every input", "[atmosphere]") {
    const auto base = optics();
    const environment::AtmosphereTableSizes base_sizes;
    const auto h0 = environment::atmosphere_input_hash(base, base_sizes);
    CHECK(h0 == environment::atmosphere_input_hash(optics(), base_sizes));

    std::vector<std::function<void(config::AtmosphereOpticsConfig&)>> config_changes{
        [](auto& c) { c.top_altitude_m += 1.0; },
        [](auto& c) { c.ground_altitude_m -= 1.0; },
        [](auto& c) { c.aerosol_optical_depth_550nm *= 1.01; },
        [](auto& c) { c.aerosol_angstrom_exponent += 0.01; },
        [](auto& c) { c.aerosol_single_scattering_albedo -= 0.01; },
        [](auto& c) { c.aerosol_scale_height_m += 1.0; },
        [](auto& c) { c.mie_phase_weight += 0.01; },
        [](auto& c) { c.mie_phase_g_forward += 0.01; },
        [](auto& c) { c.mie_phase_g_broad += 0.01; },
        [](auto& c) { c.ozone_column_du += 1.0; },
        [](auto& c) { c.ground_albedo += 0.01; },
        [](auto& c) { c.scattering_orders += 1; },
        [](auto& c) { c.multiple_scattering_tail = !c.multiple_scattering_tail; },
    };
    std::vector<std::function<void(environment::AtmosphereTableSizes&)>> size_changes{
        [](auto& s) { s.transmittance_mu *= 2; }, [](auto& s) { s.transmittance_r *= 2; },
        [](auto& s) { s.scattering_r *= 2; },     [](auto& s) { s.scattering_mu *= 2; },
        [](auto& s) { s.scattering_mu_s *= 2; },  [](auto& s) { s.scattering_nu *= 2; },
        [](auto& s) { s.irradiance_mu_s *= 2; },  [](auto& s) { s.irradiance_r *= 2; },
    };
    std::set<std::uint64_t> hashes{h0};
    for (const auto& change : config_changes) {
        auto c = base;
        change(c);
        hashes.insert(environment::atmosphere_input_hash(c, base_sizes));
    }
    for (const auto& change : size_changes) {
        auto s = base_sizes;
        change(s);
        hashes.insert(environment::atmosphere_input_hash(base, s));
    }
    CHECK(hashes.size() == 1 + config_changes.size() + size_changes.size());
}

TEST_CASE("The multiple-scattering tail is on in the shipped config and changes the tables", "[atmosphere]") {
    const auto shipped = optics();
    CHECK(shipped.multiple_scattering_tail);  // fidelity flag (earth_optics.toml), on by default
    REQUIRE(shipped.scattering_orders >= 3);  // the tail needs two computed multiple-scattering orders
    // Small tables: only the difference between the flag's two settings is asserted here.
    environment::AtmosphereTableSizes sizes;
    sizes.transmittance_mu = 32;
    sizes.transmittance_r = 16;
    sizes.scattering_r = 4;
    sizes.scattering_mu = 8;
    sizes.scattering_mu_s = 4;
    sizes.scattering_nu = 2;
    sizes.irradiance_mu_s = 8;
    sizes.irradiance_r = 4;
    auto without = shipped;
    without.multiple_scattering_tail = false;
    const auto on = environment::precompute_atmosphere(shipped, sizes);
    const auto off = environment::precompute_atmosphere(without, sizes);
    CHECK(on.transmittance == off.transmittance);
    CHECK(on.single_mie == off.single_mie);
    CHECK(on.single_rayleigh == off.single_rayleigh);
    // The single Rayleigh part lies below the scattering table's single + multiple Rayleigh (spectrally; per sRGB
    // channel the colour fold may move either way: compared summed over the table).
    REQUIRE(on.single_rayleigh.size() == on.scattering.size());
    double single_sum = 0.0;
    double all_sum = 0.0;
    for (std::size_t i = 0; i < on.scattering.size(); i += 4) {
        for (int ch = 0; ch < 3; ++ch) {
            single_sum += on.single_rayleigh[i + ch];
            all_sum += on.scattering[i + ch];
        }
        CHECK(on.single_rayleigh[i + 3] == 0.0f);
    }
    CHECK(single_sum > 0.0);
    CHECK(single_sum < all_sum);
    REQUIRE(on.scattering.size() == off.scattering.size());
    REQUIRE(on.irradiance.size() == off.irradiance.size());
    // The tail adds the higher orders' light (spectrally non-negative; per sRGB channel the fold through the colour
    // matrix may move either way): the scattering and irradiance tables differ, and their totals grow.
    double scattering_sum_on = 0.0;
    double scattering_sum_off = 0.0;
    for (std::size_t i = 0; i < on.scattering.size(); ++i) {
        scattering_sum_on += on.scattering[i];
        scattering_sum_off += off.scattering[i];
    }
    double irradiance_sum_on = 0.0;
    double irradiance_sum_off = 0.0;
    for (std::size_t i = 0; i < on.irradiance.size(); ++i) {
        irradiance_sum_on += on.irradiance[i];
        irradiance_sum_off += off.irradiance[i];
    }
    CHECK(on.scattering != off.scattering);
    CHECK(on.irradiance != off.irradiance);
    CHECK(scattering_sum_on > scattering_sum_off);
    CHECK(irradiance_sum_on > irradiance_sum_off);
}

TEST_CASE("Input hash covers the spectral data", "[atmosphere]") {
    // The checksum of the tabulated data as stored (solar spectrum, ozone cross sections, CMFs, XYZ->sRGB, wavelengths,
    // depolarisation table, AFGL ozone profile) and the input hash of the configuration with the default sizes.
    // Pinned: a change of the data, the config, the sizes or the code version changes them - update the pins together
    // with such a change. Raw inputs only (no libm outputs), so the pins hold on every machine and a shipped cache is
    // accepted everywhere.
    const std::uint64_t checksum = environment::atmosphere_spectral_data_checksum();
    const std::uint64_t hash = environment::atmosphere_input_hash(optics(), {});
    std::printf("atmosphere spectral data checksum: 0x%016llx, input hash 0x%016llx\n",
                static_cast<unsigned long long>(checksum), static_cast<unsigned long long>(hash));
    CHECK(checksum == 0x34af73121d46b1aaull);
    CHECK(hash == 0x841ab5a6c9e9448bull);  // code version 5, channels 628 / 546 / 444 nm, continental aerosol set
}

TEST_CASE("The aerosol phase functions are normalised and follow the Mie aureole", "[atmosphere]") {
    // Two-term Henyey-Greenstein of earth_optics.toml vs the Lorenz-Mie phase function of the AERONET size
    // distributions (Dubovik et al. 2002 Table 1) at 550 nm, tools/validation/aerosol_mie_check.py: P(theta) / P(25 deg)
    // at 2 / 5 / 10 deg = 8.75 / 3.04 / 1.63 (GSFC, continental) and 18.0 / 5.65 / 2.27 (Lanai, maritime) [D].
    // Tolerance 30 % (the fit's range, -18 / +29 % at 0.5-90 deg). Bruneton's Cornette-Shanks g 0.8 gives 10.9 / 9.15 /
    // 5.49 - 3x the Mie value at 5-10 deg (the lunar aureole of look-fixes T13 fix 1) - and fails.
    struct Case {
        config::AerosolType type;
        double p2, p5, p10;
    };
    for (const Case k : {Case{config::AerosolType::continental, 8.75, 3.04, 1.63},
                         Case{config::AerosolType::maritime, 18.03, 5.65, 2.27}}) {
        const auto p = environment::atmosphere_parameters(config::load_atmosphere_optics(optics_toml, k.type));
        // normalised over the sphere: 2 pi integral of p d(cos theta) = 1 (Simpson in theta, the peak resolved)
        const int n = 20000;
        double integral = 0.0;
        for (int i = 0; i <= n; ++i) {
            const double th = constants::pi * i / n;
            const double w = (i == 0 || i == n) ? 1.0 : (i % 2 ? 4.0 : 2.0);
            integral += w * environment::mie_phase_function(p.mie_phase, std::cos(th)) * std::sin(th);
        }
        integral *= 2.0 * constants::pi * (constants::pi / n) / 3.0;
        CHECK(integral == Approx(1.0).epsilon(1e-4));
        const auto at = [&](double deg) {
            return environment::mie_phase_function(p.mie_phase, std::cos(deg * constants::deg_to_rad));
        };
        const double p25 = at(25.0);
        std::printf("%s aerosol phase P/P(25): 2 deg %.2f (Mie %.2f), 5 deg %.2f (%.2f), 10 deg %.2f (%.2f), g %.3f\n",
                    config::aerosol_type_name(k.type), at(2.0) / p25, k.p2, at(5.0) / p25, k.p5, at(10.0) / p25, k.p10,
                    environment::mie_phase_asymmetry(p.mie_phase));
        CHECK(rel_error(at(2.0) / p25, k.p2) < 0.30);
        CHECK(rel_error(at(5.0) / p25, k.p5) < 0.30);
        CHECK(rel_error(at(10.0) / p25, k.p10) < 0.30);
    }
}

TEST_CASE("The maritime weight blends the two aerosol table sets", "[atmosphere]") {
    // Small tables of both sets (single scattering): w = 0 reads the continental set alone, w = 1 the maritime set, and
    // every table read in between is the linear blend (transmittance, sky irradiance; the shader include does the same).
    environment::AtmosphereTableSizes small;
    small.transmittance_mu = 64;
    small.transmittance_r = 16;
    small.scattering_r = 8;
    small.scattering_mu = 32;
    small.scattering_mu_s = 8;
    small.scattering_nu = 4;
    const auto c = std::make_shared<const environment::AtmosphereTables>(
        precompute_timed(config::load_atmosphere_optics(optics_toml, config::AerosolType::continental), small));
    const auto m = std::make_shared<const environment::AtmosphereTables>(
        precompute_timed(config::load_atmosphere_optics(optics_toml, config::AerosolType::maritime), small));
    const environment::AtmosphereScattering both(c, m), continental(c), maritime(m);
    const math::Vec3 ground{constants::wgs84_semi_major_axis_m + 10.0, 0.0, 0.0};
    const math::Vec3 sun = math::normalized(math::Vec3{1.0, 0.0, 0.3});  // ~73 deg up; the view 2.9 deg up to the east
    const math::Vec3 east{0.0, 1.0, 0.0};
    const math::Vec3 view = math::normalized(math::Vec3{0.05, 1.0, 0.0});
    const auto close = [](const math::Vec3& a, const math::Vec3& b) {
        const auto near = [](double u, double v) { return std::abs(u - v) <= 1e-12 * std::abs(v) + 1e-300; };
        return near(a.x, b.x) && near(a.y, b.y) && near(a.z, b.z);
    };
    CHECK(close(both.sun_transmittance(ground, sun, 0.0), continental.sun_transmittance(ground, sun)));
    CHECK(close(both.sun_transmittance(ground, sun, 1.0), maritime.sun_transmittance(ground, sun)));
    const math::Vec3 tc = continental.sun_transmittance(ground, sun), tm = maritime.sun_transmittance(ground, sun);
    CHECK(close(both.sun_transmittance(ground, sun, 0.25), tc * 0.75 + tm * 0.25));
    math::Vec3 t0, t1;
    CHECK(close(both.sky_luminance(ground, view, sun, &t0, 1.0), maritime.sky_luminance(ground, view, sun, &t1)));
    CHECK(close(t0, t1));
    const auto ec = continental.sun_and_sky_illuminance(ground, east, sun);
    const auto em = maritime.sun_and_sky_illuminance(ground, east, sun);
    const auto eb = both.sun_and_sky_illuminance(ground, east, sun, 0.5);
    CHECK(close(eb.sky_lux, (ec.sky_lux + em.sky_lux) * 0.5));
    // the sets differ: near the horizon the maritime sky is bluer (Angstrom 0.6 against 1.3: less blue taken out of the
    // long path - the lever on the distant clouds' colour, look-fixes bug-horizon report)
    math::Vec3 tr;
    const math::Vec3 lc = continental.sky_luminance(ground, view, sun, &tr);
    const math::Vec3 lm = maritime.sky_luminance(ground, view, sun, &tr);
    std::printf("horizon sky B/R continental %.3f maritime %.3f\n", lc.z / lc.x, lm.z / lm.x);
    CHECK(lm.z / lm.x > lc.z / lc.x);
}
