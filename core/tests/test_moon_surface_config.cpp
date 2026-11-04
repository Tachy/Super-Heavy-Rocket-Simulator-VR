#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <random>
#include <string>

#include "starsim/config/moon_surface_config.hpp"
#include "starsim/config/vehicle_config.hpp"

using namespace starsim;

namespace {

const std::filesystem::path shipped = std::filesystem::path(STARSIM_CONFIG_DIR) / "moon_surface.toml";

void check_sfd(const config::SizeFrequency& s) {
    CHECK((s.form == "power" || s.form == "exponential"));
    CHECK(std::isfinite(s.reference_diameter_m));
    CHECK(std::isfinite(s.density_per_m2));
    CHECK(std::isfinite(s.exponent));
    CHECK(s.min_diameter_m > 0.0);
    CHECK(s.max_diameter_m > s.min_diameter_m);
    // Strictly decreasing over the valid range (logarithmic sampling, 200 points).
    double previous = s.cumulative_density_per_m2(s.min_diameter_m);
    CHECK(std::isfinite(previous));
    CHECK(previous > 0.0);
    const double ratio = std::pow(s.max_diameter_m / s.min_diameter_m, 1.0 / 200.0);
    double d = s.min_diameter_m;
    for (int i = 0; i < 200; ++i) {
        d *= ratio;
        const double n = s.cumulative_density_per_m2(d);
        CHECK(std::isfinite(n));
        CHECK(n < previous);
        previous = n;
    }
}

}  // namespace

TEST_CASE("The shipped Moon surface config loads with sources-sane values", "[moon_surface][config]") {
    const auto cfg = config::load_moon_surface_config(shipped);

    check_sfd(cfg.craters.sfd);
    check_sfd(cfg.rocks.sfd);
    CHECK(cfg.rocks.sfd.min_diameter_m <= 0.01);  // rocks down to the finest drawn scale

    const auto& c = cfg.craters;
    for (double v : {c.depth_to_diameter_fresh, c.depth_to_diameter_degraded, c.rim_height_to_diameter,
                     c.degraded_fraction}) {
        CHECK(std::isfinite(v));
    }
    CHECK(c.depth_to_diameter_degraded < c.depth_to_diameter_fresh);
    CHECK(c.degraded_fraction >= 0.0);
    CHECK(c.degraded_fraction <= 1.0);

    const auto& r = cfg.roughness;
    CHECK(std::isfinite(r.rms_height_at_baseline_m));
    CHECK(std::isfinite(r.baseline_m));
    CHECK(r.hurst_exponent > 0.5);
    CHECK(r.hurst_exponent < 1.0);
    CHECK(std::isfinite(r.rms_height_measured_total_m));
    CHECK(r.rms_height_at_baseline_m > 0.0);
    CHECK(r.rms_height_at_baseline_m <= r.rms_height_measured_total_m);  // residual after craters and rocks

    const auto& k = cfg.rocks;
    for (double v : {k.axis_b_to_a_mean, k.axis_c_to_a_mean, k.axis_ratio_sd, k.burial_fraction_mean,
                     k.burial_fraction_sd, k.albedo_to_regolith, k.abundance_scale}) {
        CHECK(std::isfinite(v));
    }
    CHECK(k.axis_c_to_a_mean <= k.axis_b_to_a_mean);
    CHECK(k.axis_b_to_a_mean <= 1.0);
    CHECK((k.abundance_source == "uniform" || k.abundance_source == "diviner"));
    // The abundance reference fraction is the disc area of the configured rock law over
    // [abundance_reference_min_diameter_m, max]: integral of (pi/4) D^2 |dN/dD| dD for the power form.
    REQUIRE(k.sfd.form == "power");
    const double beta = k.sfd.exponent;
    const double d0 = k.abundance_reference_min_diameter_m, d1 = k.sfd.max_diameter_m;
    const double area = std::numbers::pi / 4.0 * k.sfd.density_per_m2 * beta * std::pow(k.sfd.reference_diameter_m, beta) *
                        (std::pow(d1, 2.0 - beta) - std::pow(d0, 2.0 - beta)) / (2.0 - beta);
    CHECK(std::abs(area / k.abundance_reference_area_fraction - 1.0) < 0.02);

    const auto& h = cfg.hapke;
    for (double v : {h.w, h.b, h.c, h.bs0, h.hs, h.bc0, h.hc, h.theta_bar_rad, h.porosity_k}) {
        CHECK(std::isfinite(v));
    }
    CHECK(h.w > 0.0);
    CHECK(h.w < 1.0);
    CHECK(std::abs(h.c) <= 1.0);  // double Henyey-Greenstein weight (Hapke 2012)
    CHECK(h.theta_bar_rad > 0.0);
    CHECK(h.theta_bar_rad < 1.0);  // ~23.7 deg, read in degrees
}

TEST_CASE("A missing key names it in German", "[moon_surface][config]") {
    // Copy of the shipped file without the Hurst exponent, under a unique name (ctest runs cases in parallel); the
    // guard removes it also when an assertion fails.
    const auto path = std::filesystem::temp_directory_path() /
                      ("starsim_moon_surface_missing_key_" + std::to_string(std::random_device{}()) + ".toml");
    struct Cleanup {
        std::filesystem::path p;
        ~Cleanup() {
            std::error_code ec;
            std::filesystem::remove(p, ec);
        }
    } cleanup{path};
    {
        std::ifstream in(shipped);
        std::ofstream out(path);
        std::string line;
        while (std::getline(in, line)) {
            if (line.rfind("hurst_exponent", 0) != 0) out << line << '\n';
        }
    }
    try {
        config::load_moon_surface_config(path);
        FAIL("loader accepted a file without hurst_exponent");
    } catch (const config::ConfigError& e) {
        const std::string message = e.what();
        CHECK_THAT(message, Catch::Matchers::ContainsSubstring("hurst_exponent"));
        CHECK_THAT(message, Catch::Matchers::ContainsSubstring("fehlt"));
    }
}
