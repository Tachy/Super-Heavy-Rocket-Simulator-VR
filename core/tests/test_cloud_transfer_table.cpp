// Monte Carlo transfer table of the multiply scattered cloud light and the source built on it (M8, Task 8b).

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/environment/cloud_optics.hpp"
#include "starsim/environment/cloud_transfer_table.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using namespace starsim::environment;
using Catch::Approx;

namespace {

namespace c = constants;

template <class... Args>
void report(const char* format, Args... args) {
    if (std::getenv("STARSIM_VERBOSE")) std::printf(format, args...);
}

std::filesystem::path table_file() {
    return std::filesystem::path(STARSIM_CONFIG_DIR) / "clouds" / "cloud_transfer_table.bin";
}

const CloudTransferTable& table() {
    static const CloudTransferTable t = load_cloud_transfer_table(table_file());
    return t;
}

// 12-point Gauss-Legendre on [-1, 1] (the radiance fixture's bin average, as test_cloud_optics.cpp).
constexpr double gl_x[12] = {-0.9815606342467192, -0.9041172563704749, -0.7699026741943047, -0.5873179542866175,
                             -0.3678314989981802, -0.1252334085114689, 0.1252334085114689,  0.3678314989981802,
                             0.5873179542866175,  0.7699026741943047,  0.9041172563704749,  0.9815606342467192};
constexpr double gl_w[12] = {0.0471753363865118, 0.1069393259953184, 0.1600783285433462, 0.2031674267230659,
                             0.2334925365383548, 0.2491470458134028, 0.2491470458134028, 0.2334925365383548,
                             0.2031674267230659, 0.1600783285433462, 0.1069393259953184, 0.0471753363865118};

// Cos-weighted average over a fixture bin (tool docstring: view zenith +-5 deg, 0..8 deg at the zenith; azimuth
// +-15 deg) of a radiance function f(mu, phi).
template <class F>
double bin_average(double view_zenith_deg, double azimuth_deg, F f) {
    const double z_lo = view_zenith_deg == 0.0 ? 0.0 : view_zenith_deg - 5.0;
    const double z_hi = view_zenith_deg == 0.0 ? 8.0 : view_zenith_deg + 5.0;
    const double a_lo = view_zenith_deg == 0.0 ? 0.0 : std::max(0.0, azimuth_deg - 15.0);
    const double a_hi = view_zenith_deg == 0.0 ? 180.0 : std::min(180.0, azimuth_deg + 15.0);
    const double mu_lo = std::cos(z_hi * c::deg_to_rad), mu_hi = std::cos(z_lo * c::deg_to_rad);
    double sum = 0.0, weight = 0.0;
    for (int i = 0; i < 12; ++i) {
        const double mu = mu_lo + (mu_hi - mu_lo) * 0.5 * (gl_x[i] + 1.0);
        for (int j = 0; j < 12; ++j) {
            const double phi = (a_lo + (a_hi - a_lo) * 0.5 * (gl_x[j] + 1.0)) * c::deg_to_rad;
            const double wt = gl_w[i] * gl_w[j] * mu;
            sum += wt * f(mu, phi);
            weight += wt;
        }
    }
    return sum / weight;
}

}  // namespace

TEST_CASE("Cloud transfer table loads with its node maps", "[weather]") {
    const CloudTransferTable& t = table();
    CHECK(t.depth_nodes == 22);
    CHECK(t.mu_nodes == 16);
    CHECK(t.theta_nodes == 48);
    CHECK(t.psi_nodes == 13);
    CHECK(t.photons > 100000000u);
    // Made with the project's phase function: its Draine weight (the view ray's extinction factor) is the core's.
    CHECK(std::abs(t.draine_weight - cloud_phase_parameters(2.0 * c::cloud_droplet_effective_radius_m).draine_weight) <
          1e-12);
    CHECK(cloud_transfer_view_extinction(t) == t.draine_weight);
    CHECK(t.command.find("cloud_slab_monte_carlo.py") != std::string::npos);
    CHECK(t.values.size() == 22u * 22u * 16u * 48u * 13u);
    // The maps put the nodes at integer coordinates: depth 0 / 0.25 / 255.75 / infinity, theta pi, mu0 1, psi pi.
    CHECK(std::abs(cloud_transfer_depth_coordinate(t, 0.0)) < 1e-12);
    CHECK(std::abs(cloud_transfer_depth_coordinate(t, 0.25) - 2.0) < 1e-12);
    CHECK(std::abs(cloud_transfer_depth_coordinate(t, 255.75) - 20.0) < 1e-9);
    CHECK(cloud_transfer_depth_coordinate(t, std::numeric_limits<double>::infinity()) == 21.0);
    CHECK(std::abs(cloud_transfer_theta_coordinate(t, c::pi) - 47.0) < 1e-9);
    CHECK(cloud_transfer_mu_coordinate(t, 1.0) == 15.0);
    CHECK(cloud_transfer_mu_coordinate(t, 1.0 / 256.0) == 0.0);
    CHECK(cloud_transfer_psi_coordinate(t, c::pi) == 12.0);
    // A foreign or short file is refused.
    const std::filesystem::path bad =
        std::filesystem::temp_directory_path() / "starsim_test_cloud_transfer_table_truncated.bin";
    {
        std::ifstream in(table_file(), std::ios::binary);
        std::vector<char> head(1000);
        in.read(head.data(), static_cast<std::streamsize>(head.size()));
        std::ofstream out(bad, std::ios::binary);
        out.write(head.data(), static_cast<std::streamsize>(head.size()));
    }
    CHECK_THROWS_AS(load_cloud_transfer_table(bad), std::runtime_error);
    std::filesystem::remove(bad);
    CHECK_THROWS_AS(load_cloud_transfer_table(std::filesystem::path(STARSIM_CONFIG_DIR) / "launch_sites.toml"),
                    std::runtime_error);
}

TEST_CASE("Cloud transfer table statistics and GPU packing", "[weather]") {
    const CloudTransferTable& t = table();
    const CloudTransferTable sigma =
        load_cloud_transfer_table(std::filesystem::path(STARSIM_WEATHER_FIXTURE_DIR) / "cloud_transfer_table_sigma.bin");
    REQUIRE(sigma.values.size() == t.values.size());
    // 1-sigma per entry, relative, over the entries that carry light (J > 1e-3 / sr); the median and the 99th
    // percentile are reported, the bound is the measured one rounded up (median 0.48 %, p99 8.6 %: the p99 entries are
    // the dim ones near 1e-3 at the slab top).
    std::vector<double> rel;
    for (std::size_t i = 0; i < t.values.size(); ++i) {
        CHECK(t.values[i] >= 0.0f);
        if (t.values[i] > 1e-3f) rel.push_back(sigma.values[i] / t.values[i]);
    }
    std::sort(rel.begin(), rel.end());
    const double median = rel[rel.size() / 2], p99 = rel[rel.size() * 99 / 100];
    report("transfer table: %zu entries > 1e-3, relative sigma median %.4f, p99 %.4f, max %.4f\n", rel.size(), median,
           p99, rel.back());
    CHECK(median < 0.01);
    CHECK(p99 < 0.10);
    // GPU form: the RGBA of a texel are the four (mu0, b) corners of its cell.
    const CloudTransferTexels gpu = cloud_transfer_gpu_texels(t);
    CHECK(gpu.width == 15 * 48);
    CHECK(gpu.height == 21 * 13);
    CHECK(gpu.depth == 22);
    const int a = 5, b = 7, m = 3, th = 11, ps = 4;
    const std::size_t texel =
        (static_cast<std::size_t>(a) * gpu.height + (b * 13 + ps)) * gpu.width + (m * 48 + th);
    const auto value = [&t](int ia, int ib, int im) {
        return t.values[(((static_cast<std::size_t>(ia) * 22 + ib) * 16 + im) * 48 + th) * 13 + ps];
    };
    CHECK(half_to_float(gpu.rgba_half[4 * texel + 0]) == value(a, b, m));
    CHECK(half_to_float(gpu.rgba_half[4 * texel + 1]) == value(a, b, m + 1));
    CHECK(half_to_float(gpu.rgba_half[4 * texel + 2]) == value(a, b + 1, m));
    CHECK(half_to_float(gpu.rgba_half[4 * texel + 3]) == value(a, b + 1, m + 1));
}

TEST_CASE("The Monte Carlo source reproduces the Monte Carlo radiance", "[weather]") {
    // Fixture: cloud_slab_radiance_mc.csv (1e7 photons per case, escape binning: independent of the table's
    // collision tally and seeds). Every bin within 3 x its statistical error or 5 %, whichever is larger - Sun
    // overhead and at 60 deg, including the forward bins where the closed form is -56 %.
    std::ifstream in(std::filesystem::path(STARSIM_WEATHER_FIXTURE_DIR) / "cloud_slab_radiance_mc.csv");
    REQUIRE(in.good());
    std::string line;
    int rows = 0;
    double worst = 0.0, worst_closed = 0.0;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] < '0' || line[0] > '9') continue;
        std::istringstream s(line);
        double sun_zenith = 0, tau = 0, view_zenith = 0, azimuth = 0, mc = 0, rel_sigma = 0;
        char comma = 0;
        s >> sun_zenith >> comma >> tau >> comma >> view_zenith >> comma >> azimuth >> comma >> mc >> comma >> rel_sigma;
        REQUIRE(s);
        const double mu0 = std::cos(sun_zenith * c::deg_to_rad);
        const double model = bin_average(view_zenith, azimuth, [&](double mu, double phi) {
            return slab_reflected_radiance_mc(table(), tau, mu0, mu, phi);
        });
        const double closed = bin_average(view_zenith, azimuth, [&](double mu, double phi) {
            return slab_reflected_radiance(tau, mu0, mu, phi);
        });
        const double error = model / mc - 1.0;
        report("sun %2.0f tau %2.0f view %2.0f az %3.0f MC %.5f (+-%.1f %%) table %.5f (%+.1f %%) closed %+.1f %%\n",
               sun_zenith, tau, view_zenith, azimuth, mc, 100.0 * rel_sigma, model, 100.0 * error,
               100.0 * (closed / mc - 1.0));
        CHECK(std::abs(error) < std::max(0.05, 3.0 * rel_sigma));
        worst = std::max(worst, std::abs(error));
        worst_closed = std::max(worst_closed, std::abs(closed / mc - 1.0));
        ++rows;
    }
    CHECK(rows == 42);
    report("Monte Carlo source vs MC radiance: %d bins, worst %.1f %% (closed form %.1f %%)\n", rows, 100.0 * worst,
           100.0 * worst_closed);
}

TEST_CASE("The Monte Carlo source gives the Monte Carlo slab albedo", "[weather]") {
    // Fixture: cloud_slab_albedo_mc.csv (tools/validation/cloud_slab_monte_carlo.py --albedo, 2e6 photons per case).
    std::ifstream in(std::filesystem::path(STARSIM_WEATHER_FIXTURE_DIR) / "cloud_slab_albedo_mc.csv");
    REQUIRE(in.good());
    std::string line;
    int rows = 0;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] < '0' || line[0] > '9') continue;
        std::istringstream s(line);
        double mu0 = 0, tau = 0, albedo = 0;
        char comma = 0;
        s >> mu0 >> comma >> tau >> comma >> albedo;
        REQUIRE(s);
        const double model = slab_reflectance_mc(table(), tau, mu0);
        report("albedo mu0 %.1f tau %2.0f MC %.4f table %.4f (%+.1f %%) closed %+.1f %%\n", mu0, tau, albedo, model,
               100.0 * (model / albedo - 1.0), 100.0 * (slab_reflectance(tau, mu0) / albedo - 1.0));
        CHECK(std::abs(model / albedo - 1.0) < 0.05);
        ++rows;
    }
    CHECK(rows == 10);
}

TEST_CASE("The Monte Carlo source conserves energy in a semi-infinite slab", "[weather]") {
    // Conservative scattering, no bottom: everything comes back out (R = 1).
    const double inf = std::numeric_limits<double>::infinity();
    for (double mu0 : {1.0, 0.5, 0.2, 0.1}) {
        const double r = slab_reflectance_mc(table(), inf, mu0);
        report("semi-infinite R(mu0 %.1f) = %.4f (closed form %.4f at tau 1e5)\n", mu0, r,
               slab_reflectance(1e5, mu0));
        CHECK(std::abs(r - 1.0) < 0.01);
    }
}

TEST_CASE("The Monte Carlo source of a uniform slab is the column problem", "[weather]") {
    // The 3D light's slab limit (Task 8 amendment): tau_s = t / mu0 gives mu_e = mu0, the axis is up, the column the
    // slab's - the source is single scattering plus the table at (t, tau* - t, mu0); integrated along an upward ray it
    // is slab_reflected_radiance_mc.
    const math::Vec3 up{0.0, 0.0, 1.0};
    const double tau_star = 12.0;
    for (double mu0 : {1.0, 0.6, 0.2}) {
        const math::Vec3 sun{std::sqrt(1.0 - mu0 * mu0), 0.0, mu0};
        for (double mu : {0.9, 0.4}) {
            for (double phi : {0.0, 1.2, c::pi}) {
                const double sv = std::sqrt(1.0 - mu * mu);
                const math::Vec3 view{-sv * std::cos(phi), -sv * std::sin(phi), mu};
                for (double t : {0.0, 0.3, 4.0, 11.0}) {
                    const double j = cloud_scattering_source_mc(table(), sun, view, up, t / mu0, t, tau_star);
                    const CloudSunFrameAngles angles = cloud_sun_frame_angles(sun * -1.0, view, up);
                    const double expected =
                        cloud_phase_function(-math::dot(sun, view)) * std::exp(-t / mu0) +
                        cloud_transfer_lookup(table(), t, tau_star - t, mu0, angles.theta_rad, angles.psi_rad);
                    CHECK(std::abs(j - expected) <= 1e-12 * std::max(1.0, expected));
                }
            }
        }
    }
    // Integrated along the ray out of the top with exp(-w_D t / mu) (midpoint rule in t, fine) = the slab radiance.
    for (double mu0 : {1.0, 0.5}) {
        const math::Vec3 sun{std::sqrt(1.0 - mu0 * mu0), 0.0, mu0};
        const double mu = 0.7, phi = 0.5, sv = std::sqrt(1.0 - mu * mu);
        // Relative azimuth phi between the horizontal propagations of sunlight (-sun: -x) and the reflected light.
        const math::Vec3 view{-sv * std::cos(phi), sv * std::sin(phi), mu};
        double radiance = 0.0;
        const int steps = 40000;
        for (int i = 0; i < steps; ++i) {
            const double t = (i + 0.5) * tau_star / steps;
            radiance += cloud_scattering_source_mc(table(), sun, view, up, t / mu0, t, tau_star) *
                        std::exp(-cloud_transfer_view_extinction(table()) * t / mu) * (tau_star / steps) / mu;
        }
        const double closed = slab_reflected_radiance_mc(table(), tau_star, mu0, mu, phi);
        report("uniform slab mu0 %.1f: ray integral %.6f, slab_reflected_radiance_mc %.6f\n", mu0, radiance, closed);
        CHECK(std::abs(radiance / closed - 1.0) < 0.002);
    }
}

TEST_CASE("The Monte Carlo slab transmittance lights the ground under a deck", "[weather]") {
    // Total beam transmittance (direct + diffuse) = 1 - albedo of the Monte Carlo fixture (2e6 photons, conservative),
    // within 3 % of T or 0.005.
    std::ifstream in(std::filesystem::path(STARSIM_WEATHER_FIXTURE_DIR) / "cloud_slab_albedo_mc.csv");
    REQUIRE(in.good());
    std::string line;
    int rows = 0;
    const CloudSlabTransmittanceLut lut = make_cloud_slab_transmittance_lut(table());
    while (std::getline(in, line)) {
        if (line.empty() || line[0] < '0' || line[0] > '9') continue;
        std::istringstream s(line);
        double mu0 = 0, tau = 0, albedo = 0;
        char comma = 0;
        s >> mu0 >> comma >> tau >> comma >> albedo;
        REQUIRE(s);
        const double expected = 1.0 - albedo;
        const double direct = slab_total_transmittance_mc(table(), tau, mu0);
        const double tabulated = cloud_slab_total_transmittance(lut, tau, mu0);
        const double closed = two_stream_beam_transmittance(tau, cloud_asymmetry_g(), mu0);
        report("T_total mu0 %.1f tau %2.0f MC %.4f model %.4f LUT %.4f (%+.1f %%) delta-Eddington %.4f (%+.1f %%)\n", mu0,
               tau, expected, direct, tabulated, 100.0 * (tabulated / expected - 1.0), closed,
               100.0 * (closed / expected - 1.0));
        CHECK(std::abs(direct - expected) <= std::max(0.03 * expected, 0.005));
        CHECK(std::abs(tabulated - expected) <= std::max(0.03 * expected, 0.005));
        ++rows;
    }
    CHECK(rows == 10);
    // The LUT: 1 without cloud, monotone in tau, the asymptote continuous at its last node.
    for (double mu0 : {0.02, 0.1, 0.5, 1.0}) {
        CHECK(cloud_slab_total_transmittance(lut, 0.0, mu0) == Approx(1.0).margin(1e-9));
        double previous = 1.0;
        for (double tau = 0.05; tau < 3000.0; tau *= 1.1) {
            const double t = cloud_slab_total_transmittance(lut, tau, mu0);
            CHECK(t <= previous + 1e-9);
            CHECK(t >= 0.0);
            previous = t;
        }
        const double at = cloud_slab_total_transmittance(lut, CloudSlabTransmittanceLut::depth_max, mu0);
        CHECK(cloud_slab_total_transmittance(lut, CloudSlabTransmittanceLut::depth_max * 1.0001, mu0) ==
              Approx(at).epsilon(1e-3));
    }
    // Delta-Eddington against the table over the used range (tau 0.5-100, Sun 5-90 deg up): reported (the shaders
    // and the CPU take the table).
    double worst = 0.0;
    for (double mu0 : {0.087, 0.25, 0.5, 0.75, 1.0}) {
        for (double tau : {0.5, 1.0, 3.0, 10.0, 30.0, 100.0}) {
            const double mc = cloud_slab_total_transmittance(lut, tau, mu0);
            worst = std::max(worst, std::abs(two_stream_beam_transmittance(tau, cloud_asymmetry_g(), mu0) / mc - 1.0));
        }
    }
    report("delta-Eddington beam transmittance vs the Monte Carlo table: worst %.1f %%\n", 100.0 * worst);
    // The sky's diffuse transmittance (two-stream, cloud_sky_factor) against the table's white-sky transmittance
    // 2 int T_total(tau, mu) mu dmu (an isotropic sky): reported and bounded.
    double sky_worst = 0.0;
    for (double tau : {1.0, 3.0, 10.0, 30.0, 100.0}) {
        double white = 0.0;
        for (int i = 0; i < 12; ++i) {
            const double mu = 0.5 * (gl_x[i] + 1.0);
            white += gl_w[i] * mu * cloud_slab_total_transmittance(lut, tau, mu);  // 2 x (1/2 sum w mu T)
        }
        const double two_stream = two_stream_diffuse_transmittance(tau, cloud_asymmetry_g());
        report("white-sky T tau %3.0f: Monte Carlo %.4f two-stream %.4f (%+.1f %%)\n", tau, white, two_stream,
               100.0 * (two_stream / white - 1.0));
        sky_worst = std::max(sky_worst, std::abs(two_stream / white - 1.0));
    }
    CHECK(sky_worst < 0.10);
    // Kasten & Czeplak 1980 (Solar Energy 24, 177): overcast global irradiance / clear-sky 0.25 (Hamburg, all Sun
    // elevations). Overcast deck tau 30 (cloud_optics test), clear-sky diffuse share 0.15 [E] of the global at Sun 45 deg:
    // the Sun's total transmittance + the sky's diffuse one in the 0.1-0.3 range of overcast skies.
    const double mu45 = std::cos(45.0 * c::deg_to_rad);
    const double ratio = 0.85 * cloud_slab_total_transmittance(lut, 30.0, mu45) +
                         0.15 * two_stream_diffuse_transmittance(30.0, cloud_asymmetry_g());
    report("overcast tau 30, Sun 45 deg: ground / clear sky %.3f (Kasten & Czeplak 0.25)\n", ratio);
    CHECK(ratio > 0.2);
    CHECK(ratio < 0.3);
}

TEST_CASE("The Sun's diffuse light under a slab takes the Monte Carlo table", "[weather]") {
    const CloudSlabTransmittanceLut lut = make_cloud_slab_transmittance_lut(table());
    const WeatherSnapshot slab = WeatherSnapshot::uniform_slab(19, 36, 1000.0, 2000.0, 2e-4, 1.0);
    const double latitude = 30.0 * c::deg_to_rad, longitude = 45.0 * c::deg_to_rad;
    const math::Vec3 position = frames::geodetic_to_fixed({latitude, longitude, 0.0});
    const math::Vec3 up{std::cos(latitude) * std::cos(longitude), std::cos(latitude) * std::sin(longitude),
                        std::sin(latitude)};
    const math::Vec3 east{-std::sin(longitude), std::cos(longitude), 0.0};
    const double tau = cloud_column_optical_depth_above(slab, latitude, longitude, 0.0);
    for (double zenith_deg : {0.0, 45.0, 80.0}) {
        const double z = zenith_deg * c::deg_to_rad, mu0 = std::cos(z);
        const double got = cloud_sun_diffuse_transmittance(slab, &lut, position, up * mu0 + east * std::sin(z));
        report("slab tau %.1f, Sun %.0f deg from the zenith: diffuse %.4f\n", tau, zenith_deg, got);
        CHECK(got == Approx(cloud_slab_total_transmittance(lut, tau, mu0) - std::exp(-tau / mu0)).epsilon(1e-6));
    }
}
