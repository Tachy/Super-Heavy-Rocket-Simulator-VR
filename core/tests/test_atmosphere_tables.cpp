// Precomputed atmosphere tables (M8 atmosphere, Task 2): transmittance and single scattering of the Bruneton port
// against the independent brute-force integrator in atmosphere_reference.hpp.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <functional>
#include <set>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "atmosphere_reference.hpp"
#include "starsim/config/atmosphere_optics_config.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/environment/atmosphere_tables.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using Catch::Approx;
namespace ref = starsim::test::atmosphere_reference;

namespace {

const std::filesystem::path optics_toml = std::filesystem::path(STARSIM_CONFIG_DIR) / "atmosphere" / "earth_optics.toml";

config::AtmosphereOpticsConfig optics() { return config::load_atmosphere_optics(optics_toml); }

environment::AtmosphereTables precompute_timed(const config::AtmosphereOpticsConfig& c,
                                               const environment::AtmosphereTableSizes& sizes = {}) {
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

TEST_CASE("Single scattering matches a brute-force raymarch", "[atmosphere]") {
    const auto c = optics();
    const environment::AtmosphereTableSizes sizes;
    const auto tables = precompute_timed(c, sizes);
    const ref::Model model{c};
    const int w = sizes.scattering_width(), h = sizes.scattering_height();

    // Texels (x = nu * mu_s_size + mu_s, mu, r). mu index 64 is the zenith and 127 the horizon (rays to the top),
    // 63 the nadir and 0 the horizon (rays to the ground). Cells: from the ground 15 deg up and 1.1 deg up (the
    // near-horizon sky), from 270 m 0.8 deg up, from -45 m just above the model horizon (a grazing ray), ground-hitting
    // rays from 795 m and 6.6 km, an upward ray at 42 km and a limb ray from the top. Measured max 1.2e-3 (Bruneton's
    // 50 uniform samples and 256 x 64 transmittance: 0.75).
    struct Texel {
        int nu, mu_s, mu, r;
    };
    const std::vector<Texel> texels{{5, 28, 80, 0},  {7, 24, 120, 0}, {6, 26, 118, 2}, {6, 26, 126, 1},
                                    {2, 30, 20, 3},  {4, 26, 40, 8},  {3, 22, 70, 20}, {6, 28, 110, 31}};
    double max_error = 0.0;
    int lookups_checked = 0;
    for (const auto& t : texels) {
        const int x = t.nu * sizes.scattering_mu_s + t.mu_s;
        const auto cell = environment::scattering_cell_at_texel(tables.parameters, sizes, x, t.mu, t.r);
        const auto expected = ref::single_scattering(model, cell.r_m, cell.mu, cell.mu_s, cell.nu);
        INFO("texel r " << t.r << " mu " << t.mu << " mu_s " << t.mu_s << " nu " << t.nu << ": alt "
                        << cell.r_m - constants::wgs84_semi_major_axis_m << " mu " << cell.mu << " mu_s " << cell.mu_s
                        << " nu " << cell.nu << " ground " << cell.ray_r_mu_intersects_ground);
        REQUIRE(expected.sun_near_horizon_samples == 0);  // the reference has no penumbra: test clean cells only
        const std::size_t i = (static_cast<std::size_t>(t.r) * h + t.mu) * w + x;
        for (int ch = 0; ch < 3; ++ch) {
            const double er = rel_error(tables.scattering[4 * i + ch], expected.rayleigh[ch]);
            const double em = rel_error(tables.single_mie[4 * i + ch], expected.mie[ch]);
            max_error = std::max({max_error, er, em});
            INFO("ch " << ch << " rayleigh " << tables.scattering[4 * i + ch] << " ref " << expected.rayleigh[ch]
                       << " mie " << tables.single_mie[4 * i + ch] << " ref " << expected.mie[ch]);
            CHECK(er < 0.01);
            CHECK(em < 0.01);
        }
        CHECK(tables.scattering[4 * i + 3] == Approx(tables.single_mie[4 * i]).epsilon(1e-6));  // A = Mie red
        // The filtered lookup at a texel centre returns the texel (when nu was not clamped to its range).
        if (std::abs(cell.nu - (2.0 * t.nu / (sizes.scattering_nu - 1) - 1.0)) < 1e-12) {
            ++lookups_checked;
            const auto looked_up = environment::lookup_scattering(tables, tables.scattering, cell);
            for (int ch = 0; ch < 4; ++ch) CHECK(looked_up[ch] == Approx(tables.scattering[4 * i + ch]).epsilon(1e-5));
        }
    }
    CHECK(lookups_checked > 0);
    std::printf("single scattering: max relative error %.3e\n", max_error);
}

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
    CHECK(p.mu_s_min == Approx(std::cos(102.0 * constants::deg_to_rad)));
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
        [](auto& c) { c.mie_phase_g += 0.01; },
        [](auto& c) { c.ozone_column_du += 1.0; },
        [](auto& c) { c.ground_albedo += 0.01; },
        [](auto& c) { c.scattering_orders += 1; },
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
