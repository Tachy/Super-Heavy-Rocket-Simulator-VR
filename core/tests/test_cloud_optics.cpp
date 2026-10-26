// Cloud optics, coverage-preserving noise and the CPU cloud light (M8 weather and clouds, Task 4).

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/environment/cloud_noise.hpp"
#include "starsim/environment/cloud_optics.hpp"
#include "starsim/environment/weather_snapshot.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using namespace starsim::environment;
using Catch::Approx;

namespace {

namespace c = constants;

// Measured numbers for the reports, printed with STARSIM_VERBOSE set only.
template <class... Args>
void report(const char* format, Args... args) {
    if (std::getenv("STARSIM_VERBOSE")) std::printf(format, args...);
}

// Integral of f(cos theta) over the sphere, composite Gauss-Legendre in theta with pieces refined toward the forward
// peak (the Henyey-Greenstein lobe of g ~0.995 is ~0.1 deg wide).
template <class F>
double sphere_integral(F f) {
    static constexpr double x[8] = {-0.9602898564975363, -0.7966664774136267, -0.5255324099163290,
                                    -0.1834346424956498, 0.1834346424956498,  0.5255324099163290,
                                    0.7966664774136267,  0.9602898564975363};
    static constexpr double w[8] = {0.1012285362903763, 0.2223810344533745, 0.3137066458778873, 0.3626837833783620,
                                    0.3626837833783620, 0.3137066458778873, 0.2223810344533745, 0.1012285362903763};
    double sum = 0.0;
    double a = 0.0;
    for (double b = 1e-5; a < c::pi; b = std::min(c::pi, b * 1.25)) {
        const int pieces = 4;
        for (int k = 0; k < pieces; ++k) {
            const double lo = a + (b - a) * k / pieces, hi = a + (b - a) * (k + 1) / pieces;
            for (int i = 0; i < 8; ++i) {
                const double theta = 0.5 * (lo + hi) + 0.5 * (hi - lo) * x[i];
                sum += 0.5 * (hi - lo) * w[i] * f(std::cos(theta)) * std::sin(theta);
            }
        }
        a = b;
    }
    return c::two_pi * sum;
}

// Local east and up at a geodetic position (ITRF).
struct LocalFrame {
    math::Vec3 position, up, east;
};

LocalFrame local_frame(double latitude_rad, double longitude_rad, double height_m) {
    LocalFrame f;
    f.position = frames::geodetic_to_fixed({latitude_rad, longitude_rad, height_m});
    f.up = {std::cos(latitude_rad) * std::cos(longitude_rad), std::cos(latitude_rad) * std::sin(longitude_rad),
            std::sin(latitude_rad)};
    f.east = {-std::sin(longitude_rad), std::cos(longitude_rad), 0.0};
    return f;
}

math::Vec3 sun_at_zenith_angle(const LocalFrame& f, double zenith_rad) {
    return f.up * std::cos(zenith_rad) + f.east * std::sin(zenith_rad);
}

// Trilinear (GPU-style, wrapping) sample of the shape texture's R channel at texel coordinates (texel centres at
// integer + 0.5), in [0, 1].
float filtered_shape_value(const CloudNoise& noise, double x, double y, double z) {
    return static_cast<float>(sample_noise_trilinear(noise.shape_rgba, noise.shape_size, 0, {x, y, z}));
}

// The fractal shape value (cloud_noise.hpp) and the width noise at uniform random positions in the shape tile (as the
// shader samples: any scale and offset), for a detail state.
struct ShapeSample {
    double value, width_noise;
};

std::vector<ShapeSample> shape_samples(const CloudNoise& noise, double detail_state, std::size_t count,
                                       std::uint64_t seed) {
    std::mt19937_64 rng(seed);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    const double n = noise.shape_size;
    std::vector<ShapeSample> samples(count);
    for (ShapeSample& s : samples) {
        const math::Vec3 p{unit(rng) * cloud_shape_tile_horizontal_m, unit(rng) * cloud_shape_tile_horizontal_m,
                           unit(rng) * cloud_shape_tile_vertical_m};
        const math::Vec3 texel{p.x * n / cloud_shape_tile_horizontal_m, p.y * n / cloud_shape_tile_horizontal_m,
                               p.z * n / cloud_shape_tile_vertical_m};
        s.value = cloud_shape_value(noise, sample_noise_trilinear(noise.shape_rgba, noise.shape_size, 0, texel), p,
                                    detail_state);
        s.width_noise = sample_noise_trilinear(noise.shape_rgba, noise.shape_size, 3, texel);
    }
    return samples;
}

}  // namespace

TEST_CASE("Extinction from water content", "[weather]") {
    // 0.3 g/m^3 liquid: 3 x 3e-4 / (2 x 1000 x 10e-6) = 0.045 /m.
    CHECK(cloud_extinction_per_m(3e-4, 0.0) == Approx(0.045).epsilon(1e-12));
    // Ice: 3 IWC / (2 x 917 x 30e-6); both phases add.
    const double ice = 3.0 * 1e-5 / (2.0 * 917.0 * 30e-6);
    CHECK(cloud_extinction_per_m(0.0, 1e-5) == Approx(ice).epsilon(1e-12));
    CHECK(cloud_extinction_per_m(3e-4, 1e-5) == Approx(0.045 + ice).epsilon(1e-12));
    CHECK(cloud_extinction_per_m(0.0, 0.0) == 0.0);
}

TEST_CASE("Phase function is normalised and forward-peaked", "[weather]") {
    const double norm = sphere_integral([](double mu) { return cloud_phase_function(mu); });
    const double g = sphere_integral([](double mu) { return mu * cloud_phase_function(mu); });
    CHECK(norm == Approx(1.0).margin(1e-3));
    // Mie for 10 um droplets in the visible: g ~0.85-0.87 (Hansen & Travis 1974, Space Sci. Rev. 16, 527).
    CHECK(g > 0.84);
    CHECK(g < 0.88);
    CHECK(cloud_asymmetry_g() == Approx(g).margin(1e-4));
    CHECK(cloud_phase_function(1.0) > 1000.0 * cloud_phase_function(0.0));
}

TEST_CASE("Slab reflectance agrees with two-stream", "[weather]") {
    // Model: the TMS source (delta-Eddington diffuse field + exact phase function on the scaled depths,
    // cloud_optics.hpp) integrated along view rays (source-function method): its albedo is not the two-stream flux.
    // Tolerances are the measured errors against delta-Eddington (task 4 fix round 1): Sun overhead -3..-5 % (within
    // 6 %); Sun at 60 deg -12..-18 % for tau >= 10 (within 19 %) and -25 % for tau 1 / 3 (within 27 %), the model too
    // dark (the smooth Eddington field misses forward multiple scattering toward grazing exit). The reference itself
    // was checked with tools/validation/cloud_slab_monte_carlo.py: delta-Eddington is within 7 % (mu0 = 1) / 10 %
    // (mu0 = 0.5) of the Monte Carlo truth, within 1 % for tau >= 30. Model constants are not tuned. Follow-up if
    // oblique light renders too dark: a Monte-Carlo transfer table (Bouthors et al. 2008).
    const double g = cloud_asymmetry_g();
    const bool verbose = std::getenv("STARSIM_VERBOSE") != nullptr;
    for (double zenith_deg : {0.0, 60.0}) {
        const double mu0 = std::cos(zenith_deg * c::deg_to_rad);
        for (double tau : {1.0, 3.0, 10.0, 30.0, 50.0}) {
            const double model = slab_reflectance(tau, mu0);
            const double reference = two_stream_beam_reflectance(tau, g, mu0);
            if (verbose) {
                report("zenith %4.0f tau %4.0f model %.4f two-stream %.4f (%+.1f %%)\n", zenith_deg, tau, model,
                            reference, 100.0 * (model / reference - 1.0));
            }
            const double tolerance = zenith_deg == 0.0 ? 0.06 : tau >= 10.0 ? 0.19 : 0.27;
            CHECK(model == Approx(reference).epsilon(tolerance));
        }
    }
}

TEST_CASE("The scattering source integrates to the slab radiance", "[weather]") {
    // cloud_scattering_source (what the shader ports) integrated numerically along an upward view ray out of a slab
    // (attenuation on the scaled depth) equals the closed form slab_reflected_radiance.
    const double k = cloud_transport_scale();
    for (const auto& [tau, mu0, mu, azimuth] : {std::array<double, 4>{10.0, 0.5, 0.7, 1.0},
                                                std::array<double, 4>{1.0, 1.0, 0.3, 0.0},
                                                std::array<double, 4>{30.0, 0.8, 0.95, 3.0}}) {
        const double cos_theta =
            std::sqrt(1.0 - mu0 * mu0) * std::sqrt(1.0 - mu * mu) * std::cos(azimuth) - mu0 * mu;
        const int steps = 200000;
        const double dt = tau / steps;
        double radiance = 0.0;
        for (int i = 0; i < steps; ++i) {
            const double t = (i + 0.5) * dt;  // vertical optical depth below the top
            radiance += cloud_scattering_source(cos_theta, mu, mu0, t / mu0, t, tau) * std::exp(-k * t / mu) * dt / mu;
        }
        CHECK(radiance == Approx(slab_reflected_radiance(tau, mu0, mu, azimuth)).epsilon(1e-6));
    }
    CHECK(cloud_scattering_source(0.5, 0.5, -0.1, 1.0, 1.0, 10.0) == 0.0);  // Sun below the horizon
}

TEST_CASE("The sky scattering source reproduces the diffuse two-stream fluxes", "[weather]") {
    // cloud_sky_scattering_source (isotropic sky light on the column top, what the shader adds to the Sun's source)
    // integrated along view rays out of the top (reflected) and out of the bottom (transmitted, plus the unscattered
    // sky seen through the slab), then over the hemisphere: the fluxes of the delta-Eddington diffuse slab within the
    // source-function method's error. Measured (task 8): R 0.1044 / 0.4780 / 0.7309 against 0.0830 / 0.4751 / 0.7309,
    // T 0.8956 / 0.5220 / 0.2691 against 0.9170 / 0.5249 / 0.2691 (tau 1 / 10 / 30); R + T = 1 (conservative). At
    // tau 1 the radiance field reflects 26 % more than the two-stream flux (whose thin-slab diffuse R is the cruder of
    // the two), hence the wider tolerance there.
    const double k = cloud_transport_scale(), g = cloud_asymmetry_g();
    static const std::array<double, 8> x = {0.0198550717512319, 0.1016667612931866, 0.2372337950418355,
                                            0.4082826787521751, 0.5917173212478249, 0.7627662049581645,
                                            0.8983332387068134, 0.9801449282487681};
    static const std::array<double, 8> w = {0.0506142681451881, 0.1111905172266872, 0.1568533229389437,
                                            0.1813418916891810, 0.1813418916891810, 0.1568533229389437,
                                            0.1111905172266872, 0.0506142681451881};
    for (double tau : {1.0, 10.0, 30.0}) {
        double up_flux = 0.0, down_flux = 0.0;
        for (std::size_t i = 0; i < x.size(); ++i) {
            const double mu = x[i];
            const int steps = 4000;
            const double dt = tau / steps;
            double reflected = 0.0, transmitted = 0.0;
            for (int s = 0; s < steps; ++s) {
                const double t = (s + 0.5) * dt;  // vertical depth below the top
                reflected += cloud_sky_scattering_source(mu, t, tau) * std::exp(-k * t / mu) * dt / mu;
                transmitted += cloud_sky_scattering_source(-mu, t, tau) * std::exp(-k * (tau - t) / mu) * dt / mu;
            }
            // The sky (radiance 1 / pi per unit irradiance) through the slab in the scaled problem: the unscattered part
            // exp(-tau / mu) plus the light kept in the forward peak (the two-stream T holds both).
            transmitted += std::exp(-k * tau / mu) / c::pi;
            up_flux += w[i] * mu * c::two_pi * reflected;
            down_flux += w[i] * mu * c::two_pi * transmitted;
        }
        INFO("tau " << tau << " R " << up_flux << " T " << down_flux);
        CHECK(up_flux == Approx(two_stream_reflectance(tau, g)).epsilon(tau < 5.0 ? 0.3 : 0.02));
        CHECK(down_flux == Approx(two_stream_diffuse_transmittance(tau, g)).epsilon(0.03));
        CHECK(up_flux + down_flux == Approx(1.0).epsilon(1e-3));
    }
}

TEST_CASE("Reflected cloud radiance agrees with Monte Carlo", "[weather]") {
    // Fixture: tools/validation/cloud_slab_monte_carlo.py --radiance 1e7 photons, bin-averaged reflected radiance of a
    // slab with the same phase function (tau 1 / 10 / 30, Sun zenith 0 / 60 deg, view zenith 0 / 30 / 60 deg, relative
    // azimuth 0 / 90 / 180 deg; statistical error <= 1.6 % per bin). The model is averaged over the same bins (12 x 12
    // Gauss-Legendre, cos-weighted). Tolerances: the measured errors (task 4 fix round 1) rounded up by the MC noise:
    //   Sun overhead: -8..+6 % -> 10 %;
    //   Sun at 60 deg, nadir view and backscatter side (azimuth 180): -19..+4 % -> 22 %; azimuth 90: -40 % -> 45 %;
    //   forward side (azimuth 0): -56 % -> 60 % (the smooth Eddington field misses the forward multiple scattering).
    // Never brighter than Monte Carlo by more than 8 % (measured max +6.4 %): first-order light is counted once.
    std::ifstream in(std::filesystem::path(STARSIM_WEATHER_FIXTURE_DIR) / "cloud_slab_radiance_mc.csv");
    REQUIRE(in.good());
    const bool verbose = std::getenv("STARSIM_VERBOSE") != nullptr;
    static constexpr double x[12] = {-0.9815606342467192, -0.9041172563704749, -0.7699026741943047,
                                     -0.5873179542866175, -0.3678314989981802, -0.1252334085114689,
                                     0.1252334085114689,  0.3678314989981802,  0.5873179542866175,
                                     0.7699026741943047,  0.9041172563704749,  0.9815606342467192};
    static constexpr double w[12] = {0.0471753363865118, 0.1069393259953184, 0.1600783285433462, 0.2031674267230659,
                                     0.2334925365383548, 0.2491470458134028, 0.2491470458134028, 0.2334925365383548,
                                     0.2031674267230659, 0.1600783285433462, 0.1069393259953184, 0.0471753363865118};
    std::string line;
    int rows = 0;
    double max_over = -1.0;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] < '0' || line[0] > '9') continue;
        std::istringstream s(line);
        double sun_zenith = 0, tau = 0, view_zenith = 0, azimuth = 0, mc = 0;
        char comma = 0;
        s >> sun_zenith >> comma >> tau >> comma >> view_zenith >> comma >> azimuth >> comma >> mc;
        REQUIRE(s);
        // Bin of the fixture (tool docstring): view zenith +-5 deg (0..8 deg at the zenith), azimuth +-15 deg.
        const double z_lo = view_zenith == 0.0 ? 0.0 : view_zenith - 5.0;
        const double z_hi = view_zenith == 0.0 ? 8.0 : view_zenith + 5.0;
        const double a_lo = view_zenith == 0.0 ? 0.0 : std::max(0.0, azimuth - 15.0);
        const double a_hi = view_zenith == 0.0 ? 180.0 : std::min(180.0, azimuth + 15.0);
        const double mu_lo = std::cos(z_hi * c::deg_to_rad), mu_hi = std::cos(z_lo * c::deg_to_rad);
        const double mu0 = std::cos(sun_zenith * c::deg_to_rad);
        double sum = 0.0, weight = 0.0;
        for (int i = 0; i < 12; ++i) {
            const double mu = mu_lo + (mu_hi - mu_lo) * 0.5 * (x[i] + 1.0);
            for (int j = 0; j < 12; ++j) {
                const double phi = (a_lo + (a_hi - a_lo) * 0.5 * (x[j] + 1.0)) * c::deg_to_rad;
                const double wt = w[i] * w[j] * mu;
                sum += wt * slab_reflected_radiance(tau, mu0, mu, phi);
                weight += wt;
            }
        }
        const double model = sum / weight, error = model / mc - 1.0;
        if (verbose) {
            report("sun %2.0f tau %2.0f view %2.0f az %3.0f MC %.5f model %.5f (%+.1f %%)\n", sun_zenith, tau,
                        view_zenith, azimuth, mc, model, 100.0 * error);
        }
        double tolerance = 0.10;
        if (sun_zenith != 0.0) {
            tolerance = view_zenith == 0.0 || azimuth == 180.0 ? 0.22 : azimuth == 90.0 ? 0.45 : 0.60;
        }
        CHECK(std::abs(error) < tolerance);
        CHECK(error < 0.08);
        max_over = std::max(max_over, error);
        ++rows;
    }
    CHECK(rows == 42);
    report("cloud radiance vs Monte Carlo: %d bins, model at most %+.1f %% above\n", rows, 100.0 * max_over);
}

TEST_CASE("An overcast deck dims the ground physically", "[weather]") {
    // Overcast skies pass 0.1-0.3 of the clear-sky global irradiance (Kasten & Czeplak 1980, Solar Energy 24, 177:
    // overcast mean G(8/8) / G(0) = 0.25, Hamburg). Tau 30 is the representative overcast deck (gives ~0.27).
    const double t = two_stream_diffuse_transmittance(30.0, cloud_asymmetry_g());
    CHECK(t > 0.1);
    CHECK(t < 0.3);
    // Tau 20 against the Monte Carlo value 0.346 of the same phase function
    // (tools/validation/cloud_slab_monte_carlo.py), within 5 %.
    CHECK(two_stream_diffuse_transmittance(20.0, cloud_asymmetry_g()) == Approx(0.346).epsilon(0.05));
    // Energy of the shader's source: a conservative semi-infinite cloud reflects everything (R = 1). Measured (task 4
    // fix round 1): 0.976 (Sun overhead), 0.901 (60 deg), 1.00004 (78.5 deg): the source loses up to 10 % (smooth
    // Eddington field) and creates less than 0.1 %.
    for (double mu0 : {1.0, 0.5, 0.2}) {
        const double r = slab_reflectance(5000.0, mu0);
        CHECK(r > 0.88);
        CHECK(r < 1.001);
    }
}

TEST_CASE("Noise coverage is preserved", "[weather]") {
    const auto start = std::chrono::steady_clock::now();
    const CloudNoise noise = generate_cloud_noise();
    const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    report("cloud noise generated in %.2f s\n", seconds);

    const int n = noise.shape_size;
    REQUIRE(noise.shape_rgba.size() == static_cast<std::size_t>(n) * n * n * 4);
    REQUIRE(noise.detail_rgba.size() ==
            static_cast<std::size_t>(noise.detail_size) * noise.detail_size * noise.detail_size * 4);
    // The shader samples the shape trilinearly (magnified, no mipmaps) at arbitrary scale and offset: the covered
    // fraction is measured on filtered values at uniform random positions (2e6 independent, fixed seed; sampling error
    // <= 3.5e-4, so 0.002) against the table entry's i / 255. The texel fraction (nearest sampling) is printed for
    // comparison: filtering narrows the distribution.
    const std::size_t texels = static_cast<std::size_t>(n) * n * n;
    std::mt19937_64 rng(42);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    std::vector<float> filtered(2000000);
    for (float& v : filtered) v = filtered_shape_value(noise, unit(rng) * n, unit(rng) * n, unit(rng) * n);
    for (double cf : {0.1, 0.5, 0.9}) {
        const long index = std::lround(cf * 255.0);
        const float t = noise.coverage_threshold[0][static_cast<std::size_t>(index)];
        std::size_t above = 0, texels_above = 0;
        for (float v : filtered) above += v > t ? 1 : 0;
        for (std::size_t i = 0; i < texels; ++i) texels_above += noise.shape_rgba[4 * i] / 255.0f > t ? 1 : 0;
        const double fraction = static_cast<double>(above) / filtered.size();
        report("cf %.1f: filtered fraction %.4f, texel fraction %.4f\n", cf, fraction,
               static_cast<double>(texels_above) / texels);
        CHECK(fraction == Approx(index / 255.0).margin(0.002));
    }
    // The fractal value of every detail state (and a fractional one, the rows interpolated as the shader's filter does):
    // the covered fraction stays cf (5e5 random positions each: sampling error <= 7e-4, so 0.003; 0.006 between rows).
    for (double state : {1.0, 1.5, 2.0, 3.0}) {
        const std::vector<ShapeSample> samples = shape_samples(noise, state, 500000, 44);
        for (double cf : {0.1, 0.5, 0.9}) {
            const int index = static_cast<int>(std::lround(cf * 255.0));
            const double t = cloud_coverage_threshold(noise, state, index);
            std::size_t above = 0;
            for (const ShapeSample& s : samples) above += s.value > t ? 1 : 0;
            const double fraction = static_cast<double>(above) / samples.size();
            report("state %.1f cf %.1f: covered fraction %.4f\n", state, cf, fraction);
            CHECK(fraction == Approx(index / 255.0).margin(state == std::floor(state) ? 0.003 : 0.006));
        }
    }
    for (const auto& table : noise.coverage_threshold) {
        CHECK(table[0] > table[1]);
        CHECK(table[255] < table[254]);
        for (int i = 1; i < 256; ++i) CHECK(table[static_cast<std::size_t>(i)] <= table[static_cast<std::size_t>(i - 1)]);
    }
    CHECK(noise.coverage_threshold[0][0] >= 1.0f);
    CHECK(noise.coverage_threshold[0][255] < 0.0f);
    // The detail levels continue the shape's spectrum as fBm: a_k / a_(k-1) = 4^-H.
    REQUIRE(noise.level_amplitude[0] > 0.0f);
    for (int k = 1; k < cloud_detail_levels; ++k) {
        CHECK(noise.level_amplitude[static_cast<std::size_t>(k)] / noise.level_amplitude[static_cast<std::size_t>(k - 1)] ==
              Approx(std::pow(4.0, -cloud_shape_hurst_exponent())).epsilon(1e-5));
    }
    report("detail amplitude %.4f, detail mean %.4f\n", noise.level_amplitude[0], noise.detail_mean);
    // The level weights: full at a footprint of 1/4 of the level's middle cell, none at 1/2, finest first.
    CHECK(cloud_detail_level_weight(0, 250.0 / 4.0) == Approx(1.0));
    CHECK(cloud_detail_level_weight(0, 250.0 / 2.0) == Approx(0.0).margin(1e-12));
    CHECK(cloud_detail_state(0.0) == Approx(cloud_detail_levels));
    CHECK(cloud_detail_state(1.0e6) == Approx(0.0).margin(1e-12));
    for (double f : {1.0, 3.0, 10.0, 30.0, 100.0}) {
        const double state = cloud_detail_state(f);
        for (int k = 0; k < cloud_detail_levels; ++k) {
            CHECK(cloud_detail_level_weight(k, f) == Approx(std::clamp(state - k, 0.0, 1.0)).margin(1e-12));
        }
    }

    // The primitives repeat with their period (opposite faces equal after the wrap).
    for (const math::Vec3 p : {math::Vec3{0.3, 1.7, 2.2}, math::Vec3{3.9, 0.1, 0.05}, math::Vec3{0.0, 2.5, 3.99}}) {
        for (const math::Vec3 shift : {math::Vec3{4, 0, 0}, math::Vec3{0, 4, 0}, math::Vec3{0, 0, 4}}) {
            CHECK(tileable_perlin(p + shift, 4) == Approx(tileable_perlin(p, 4)).margin(1e-12));
            CHECK(tileable_worley(p + shift, 4, 7u) == Approx(tileable_worley(p, 4, 7u)).margin(1e-12));
        }
    }
    // No seam in the textures: the step across the wrap (last -> first texel) is like an interior step.
    auto seam_ratio = [](const std::vector<std::uint8_t>& rgba, int size, int channel, int axis) {
        auto at = [&](int x, int y, int z) {
            return static_cast<double>(rgba[4 * ((static_cast<std::size_t>(z) * size + y) * size + x) + channel]);
        };
        double seam = 0.0, interior = 0.0;
        for (int a = 0; a < size; ++a) {
            for (int b = 0; b < size; ++b) {
                auto value = [&](int k) { return axis == 0 ? at(k, a, b) : axis == 1 ? at(a, k, b) : at(a, b, k); };
                seam += std::abs(value(size - 1) - value(0));
                interior += std::abs(value(size / 2) - value(size / 2 - 1));
            }
        }
        return seam / interior;
    };
    for (int axis = 0; axis < 3; ++axis) {
        for (int channel = 0; channel < 4; ++channel) {
            const double r = seam_ratio(noise.shape_rgba, n, channel, axis);
            CHECK(r > 0.67);
            CHECK(r < 1.5);
        }
        for (int channel = 0; channel < 3; ++channel) {
            const double r = seam_ratio(noise.detail_rgba, noise.detail_size, channel, axis);
            CHECK(r > 0.67);
            CHECK(r < 1.5);
        }
    }
    // Deterministic.
    const CloudNoise again = generate_cloud_noise();
    CHECK(again.shape_rgba == noise.shape_rgba);
    CHECK(again.detail_rgba == noise.detail_rgba);
}

TEST_CASE("A cumulus edge rises from 10 % to 90 % of its interior extinction within the edge width", "[weather]") {
    // The steepness: the shell of a cumulus is tens of metres, not the hundreds of the M8 soft range. The ramp's 10 %
    // to 90 % rise spans 0.61 of the width w(n), so in metres it stays inside the edge width, and the widest edge is
    // at most 40 m.
    const CloudNoise noise = generate_cloud_noise();
    CHECK(c::cloud_edge_width_min_m == Approx(20.0));
    CHECK(c::cloud_edge_width_max_m <= 40.0);
    auto ramp_at = [](double level) {
        double lo = 0.0, hi = 1.0;
        for (int i = 0; i < 60; ++i) (cloud_edge_ramp(0.5 * (lo + hi)) < level ? lo : hi) = 0.5 * (lo + hi);
        return 0.5 * (lo + hi);
    };
    const double rise = ramp_at(0.9) - ramp_at(0.1);  // in widths
    CHECK(rise < 1.0);
    for (double n : {0.0, 0.5, 1.0}) {
        const double width_m = c::cloud_edge_width_min_m * std::pow(c::cloud_edge_width_max_m / c::cloud_edge_width_min_m, n);
        const double rise_m = rise * width_m;  // the ramp is linear in the value, the value in m by the median increment
        report("width noise %.1f: edge %.1f m, 10..90 %% rise %.1f m (value width %.4f)\n", n, width_m, rise_m,
               cloud_edge_width_value(noise, n));
        CHECK(rise_m <= width_m);
        CHECK(rise_m <= 40.0);
    }
}

TEST_CASE("Cloud edges of varying width keep the grid-mean water at every detail state", "[weather]") {
    // density = mean / m(cf) x ramp((v - t(cf)) / w(n)) (cloud_noise.hpp, soft edges) with the fractal value v of the
    // detail state and the edge width w(n) of the width noise: over uniform random positions (5e5 per state) the mean of
    // ramp / m is 1 (the grid-mean water is kept) within 1.5 % at the table states and 2.5 % between them.
    const CloudNoise noise = generate_cloud_noise();
    report("edge width %.4f .. %.4f in shape value (%.0f .. %.0f m)\n", noise.edge_width_min_value,
           noise.edge_width_max_value, c::cloud_edge_width_min_m, c::cloud_edge_width_max_m);
    CHECK(noise.edge_width_min_value > 0.002f);
    CHECK(noise.edge_width_max_value > noise.edge_width_min_value);
    CHECK(noise.edge_width_max_value < 0.3f);
    CHECK(cloud_edge_width_value(noise, 0.0) == Approx(noise.edge_width_min_value));
    CHECK(cloud_edge_width_value(noise, 1.0) == Approx(noise.edge_width_max_value));
    for (const auto& m : noise.edge_mean_factor) {
        CHECK(m[0] == 0.0f);
        for (int i = 1; i < 256; ++i) {
            CHECK(m[static_cast<std::size_t>(i)] >= m[static_cast<std::size_t>(i - 1)]);
            CHECK(m[static_cast<std::size_t>(i)] <= i / 255.0 + 1e-6);
        }
    }
    for (double state : {0.0, 1.0, 1.5, 3.0}) {
        const std::vector<ShapeSample> samples = shape_samples(noise, state, 500000, 45);
        for (double cf : {0.05, 0.3, 0.5, 0.9, 1.0}) {
            const int index = static_cast<int>(std::lround(cf * 255.0));
            const double t = cloud_coverage_threshold(noise, state, index), m = cloud_edge_mean_factor(noise, state, index);
            double water = 0.0;
            for (const ShapeSample& s : samples) water += cloud_edge_ramp((s.value - t) / cloud_edge_width_value(noise, s.width_noise)) / m;
            water /= samples.size();
            report("state %.1f cf %.2f: m %.4f, mean water / grid mean %.4f\n", state, cf, m, water);
            CHECK(water == Approx(1.0).epsilon(state == std::floor(state) ? 0.015 : 0.025));
            CHECK(m <= index / 255.0 + 1e-6);  // the edge zone holds less water than the interior
            if (index < 255) CHECK(m < index / 255.0);
        }
    }
    CHECK(cloud_edge_ramp(-0.5) == 0.0);
    CHECK(cloud_edge_ramp(0.5) == Approx(0.5));
    CHECK(cloud_edge_ramp(2.0) == 1.0);
}

TEST_CASE("The shape prefilter keeps the covered fraction and the water at large pixel footprints", "[weather]") {
    // Task 10 fix round 1 (cloud_noise.hpp "Shape prefilter"): beyond cloud_shape_prefilter_start_m the shape value is
    // read from the box-filtered mip chain at the level lambda(f) (the shape's octaves finer than the pixel enter with
    // their mean), against the level's threshold and edge mean factor: the covered fraction stays cf and the mean water
    // the grid mean. Below the start the level is 0 - the unfiltered shape, exactly.
    const CloudNoise noise = generate_cloud_noise();
    const double start = cloud_shape_prefilter_start_m;
    const double texel_m = cloud_shape_tile_horizontal_m / noise.shape_size;
    for (double f : {0.0, 1.0, 250.0, 2000.0, start}) CHECK(cloud_shape_prefilter_level(f) == 0.0);
    CHECK(cloud_shape_prefilter_level(start * (1.0 + 1e-6)) < 1e-4);  // continuous at the start (no pop)
    CHECK(cloud_shape_prefilter_level(2.0 * start) ==
          Approx(std::min(std::log2(2.0 * start / texel_m), static_cast<double>(cloud_shape_prefilter_levels))));
    CHECK(cloud_shape_prefilter_level(1.1 * start) == Approx(std::log2(1.1) * std::log2(1.1 * start / texel_m)));
    CHECK(cloud_shape_prefilter_level(1.0e6) == cloud_shape_prefilter_levels);
    double previous = 0.0;
    for (double f = start; f < 1.0e5; f *= 1.05) {
        const double level = cloud_shape_prefilter_level(f);
        CHECK(level >= previous);
        previous = level;
    }

    // The mip chain: box filter of 2 x 2 x 2 texels (rounded), down to one texel.
    REQUIRE(noise.shape_mips.size() == 7);
    for (std::size_t l = 0; l < noise.shape_mips.size(); ++l) {
        const std::size_t size = static_cast<std::size_t>(noise.shape_size >> (l + 1));
        REQUIRE(noise.shape_mips[l].size() == size * size * size * 4);
    }
    for (int ch = 0; ch < 4; ++ch) {
        int sum = 0;
        for (int c = 0; c < 8; ++c) {
            // Level 1 texel (2, 1, 1) <- level 0 texels x 4..5, y 2..3, z 2..3.
            const std::size_t i = (static_cast<std::size_t>(2 + (c >> 2)) * noise.shape_size + 2 + ((c >> 1) & 1)) *
                                      noise.shape_size + 4 + (c & 1);
            sum += noise.shape_rgba[4 * i + static_cast<std::size_t>(ch)];
        }
        const std::size_t j = (static_cast<std::size_t>(1) * (noise.shape_size / 2) + 1) * (noise.shape_size / 2) + 2;
        CHECK(noise.shape_mips[0][4 * j + static_cast<std::size_t>(ch)] == (sum + 4) / 8);
    }
    // Level 0: the unfiltered trilinear sample and the state-0 tables, bit for bit.
    for (const math::Vec3 texel : {math::Vec3{3.3, 70.1, 5.9}, math::Vec3{127.9, 0.2, 64.5}}) {
        for (int ch = 0; ch < 4; ++ch) {
            CHECK(sample_shape_prefiltered(noise, ch, texel, 0.0) ==
                  sample_noise_trilinear(noise.shape_rgba, noise.shape_size, ch, texel));
        }
    }
    CHECK(noise.prefilter_threshold[0] == noise.coverage_threshold[0]);
    CHECK(noise.prefilter_edge_mean[0] == noise.edge_mean_factor[0]);

    // Covered fraction and mean water at table levels and between them (GPU: the mips blended linearly in the level,
    // the table rows linearly): 4e5 uniform random positions per level.
    std::mt19937_64 rng(46);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    const double n = noise.shape_size;
    for (double level : {0.125, 0.5, 1.0, 1.25, 1.625, 1.875, 2.0}) {
        std::vector<ShapeSample> samples(400000);
        for (ShapeSample& s : samples) {
            const math::Vec3 texel{unit(rng) * n, unit(rng) * n, unit(rng) * n};
            s.value = sample_shape_prefiltered(noise, 0, texel, level);
            s.width_noise = sample_shape_prefiltered(noise, 3, texel, level);
        }
        const bool on_row = level * cloud_shape_prefilter_rows_per_level == std::floor(level * cloud_shape_prefilter_rows_per_level);
        for (double cf : {0.1, 0.4, 0.9}) {
            const int index = static_cast<int>(std::lround(cf * 255.0));
            const double t = cloud_prefilter_threshold(noise, level, index);
            const double m = cloud_prefilter_edge_mean(noise, level, index);
            double covered = 0.0, water = 0.0;
            for (const ShapeSample& s : samples) {
                covered += s.value > t ? 1.0 : 0.0;
                water += cloud_edge_ramp((s.value - t) / cloud_edge_width_value(noise, s.width_noise)) / m;
            }
            covered /= samples.size();
            water /= samples.size();
            report("prefilter level %.2f cf %.2f: covered %.4f, m %.4f, mean water / grid mean %.4f\n", level, cf,
                   covered, m, water);
            CHECK(std::abs(covered - index / 255.0) <= (on_row ? 0.01 : 0.02));
            CHECK(water == Approx(1.0).epsilon(on_row ? 0.015 : 0.025));
        }
    }
}

TEST_CASE("Cloud perimeters have the observed fractal dimension", "[weather]") {
    // Lovejoy 1982 (Science 216, 185): cloud and rain areas have the area-perimeter dimension D = 1.35 +- 0.05 from
    // 1 to 1.2e6 km^2 (constants.hpp). The sub-grid shapes continue it: the cloud mask (v > t(cf), all detail levels)
    // of a horizontal 16 km x 16 km plane at 4 m, cf 0.5; box counting of the boundary pixels with boxes of 8 m to
    // 1024 m (2.1 decades, 8 sizes): N(b) ~ b^-D, D the least-squares slope; 1.25 <= D <= 1.45. The shape alone
    // (detail state 0) is printed for comparison (smooth below its 500 m octave: D near 1 at these scales).
    const CloudNoise noise = generate_cloud_noise();
    const int size = 4096;
    const double pixel_m = 4.0, height_m = 1234.0;
    auto dimension = [&](double state) {
        const double t = cloud_coverage_threshold(noise, state, 128);
        std::vector<std::uint8_t> mask(static_cast<std::size_t>(size) * size);
        const int threads = std::max(1, static_cast<int>(std::thread::hardware_concurrency()));
        std::vector<std::thread> pool;
        for (int w = 0; w < threads; ++w) {
            pool.emplace_back([&, w] {
                const double n = noise.shape_size;
                for (int y = w; y < size; y += threads) {
                    for (int x = 0; x < size; ++x) {
                        const math::Vec3 p{(x + 0.5) * pixel_m, (y + 0.5) * pixel_m, height_m};
                        const math::Vec3 texel{p.x * n / cloud_shape_tile_horizontal_m, p.y * n / cloud_shape_tile_horizontal_m,
                                               p.z * n / cloud_shape_tile_vertical_m};
                        const double v = cloud_shape_value(
                            noise, sample_noise_trilinear(noise.shape_rgba, noise.shape_size, 0, texel), p, state);
                        mask[static_cast<std::size_t>(y) * size + x] = v > t ? 1 : 0;
                    }
                }
            });
        }
        for (auto& th : pool) th.join();
        std::vector<std::uint8_t> boundary(mask.size(), 0);
        std::size_t covered = 0;
        for (int y = 0; y < size; ++y) {
            for (int x = 0; x < size; ++x) {
                const std::uint8_t m = mask[static_cast<std::size_t>(y) * size + x];
                covered += m;
                const bool edge = (x + 1 < size && mask[static_cast<std::size_t>(y) * size + x + 1] != m) ||
                                  (y + 1 < size && mask[static_cast<std::size_t>(y + 1) * size + x] != m);
                boundary[static_cast<std::size_t>(y) * size + x] = edge ? 1 : 0;
            }
        }
        double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
        int points = 0;
        for (int box = 2; box <= 256; box *= 2) {
            std::size_t count = 0;
            for (int by = 0; by < size; by += box) {
                for (int bx = 0; bx < size; bx += box) {
                    bool hit = false;
                    for (int y = by; y < by + box && !hit; ++y) {
                        for (int x = bx; x < bx + box; ++x) {
                            if (boundary[static_cast<std::size_t>(y) * size + x]) {
                                hit = true;
                                break;
                            }
                        }
                    }
                    count += hit ? 1 : 0;
                }
            }
            const double lx = std::log(box * pixel_m), ly = std::log(static_cast<double>(count));
            report("state %.0f box %5.0f m: %zu boxes\n", state, box * pixel_m, count);
            sx += lx;
            sy += ly;
            sxx += lx * lx;
            sxy += lx * ly;
            ++points;
        }
        report("state %.0f covered fraction %.3f\n", state, static_cast<double>(covered) / mask.size());
        return -(points * sxy - sx * sy) / (points * sxx - sx * sx);
    };
    const double d_shape = dimension(0.0);
    const double d = dimension(cloud_detail_levels);
    report("perimeter fractal dimension: shape alone %.3f, with the detail levels %.3f (Lovejoy 1982: 1.35)\n", d_shape, d);
    CHECK(d >= 1.25);
    CHECK(d <= 1.45);
}

TEST_CASE("Blue noise mask is uniform and blue", "[weather]") {
    const int size = 64;
    const std::vector<std::uint8_t> mask = generate_blue_noise(size);
    REQUIRE(mask.size() == static_cast<std::size_t>(size) * size);
    std::array<int, 256> histogram{};
    for (std::uint8_t v : mask) ++histogram[v];
    for (int count : histogram) CHECK(count == size * size / 256);  // every value equally often
    // Blue: the means of 4 x 4 blocks (wrapping) vary far less than white noise's ((1/12) / 16 for uniform values):
    // the error has no low frequencies.
    double sum_sq = 0.0;
    int blocks = 0;
    for (int y = 0; y < size; ++y) {
        for (int x = 0; x < size; ++x) {
            double mean = 0.0;
            for (int dy = 0; dy < 4; ++dy) {
                for (int dx = 0; dx < 4; ++dx) mean += mask[static_cast<std::size_t>((y + dy) % size) * size + (x + dx) % size];
            }
            mean = mean / 16.0 / 255.0 - 0.5;
            sum_sq += mean * mean;
            ++blocks;
        }
    }
    const double variance = sum_sq / blocks, white = 1.0 / 12.0 / 16.0;
    report("blue noise 4x4 block-mean variance %.6f (white %.6f)\n", variance, white);
    CHECK(variance < 0.25 * white);  // measured 0.18
    CHECK(generate_blue_noise(size) == mask);  // deterministic
}

TEST_CASE("3D Sun light on an isolated cumulus", "[weather]") {
    // A cumulus-shaped blob: a sphere of radius 600 m with the soft edge ramp over the outer 100 m, in-cloud extinction
    // 0.004 /m (tau 4.4 across). The Sun 30 deg above the horizon from +x. Points 50 m inside the sunlit and the shadowed
    // flank at the sphere's middle height; the camera looks at each from the Sun's side (backscatter, 150 deg).
    const double radius = 600.0, beta = 0.004, edge = 100.0;  // within cloud_edge_width_min_m .. _max_m
    const math::Vec3 sun = {std::cos(30.0 * c::deg_to_rad), 0.0, std::sin(30.0 * c::deg_to_rad)};
    auto extinction_at = [&](const math::Vec3& p) {
        return beta * cloud_edge_ramp((radius - math::norm(p)) / edge);
    };
    // Optical depth along the Sun ray by fine quadrature (the reference).
    auto exact_tau = [&](const math::Vec3& from) {
        double tau = 0.0;
        const double ds = 0.05;
        for (double s = 0.5 * ds; s < 2.0 * radius; s += ds) tau += extinction_at(from + sun * s) * ds;
        return tau;
    };
    const math::Vec3 sunlit = sun * (radius - 50.0), shadowed = sun * -(radius - 50.0);
    CloudLightMarch march;
    march.first_m = 20.0;
    march.near_length_m = 6000.0;
    march.near_samples = 6;
    auto marched = [&](const math::Vec3& from, double jitter, const CloudLightMarch& m) {
        auto along = [&](double s) { return extinction_at(from + sun * s); };
        return cloud_light_optical_depth(along, along, m, jitter);
    };
    // Unbiased over the jitter (what the per-pixel jitter averages): the mean optical depth over 4096 stratified
    // jitters equals the integral within 1 %, so the shadow side's transmittance exp(-mean tau) is Beer-Lambert along
    // the Sun ray within 2 %. Printed: the per-jitter spread and the mean transmittance E[exp(-tau)] (Jensen: brighter).
    const double tau_shadow = exact_tau(shadowed);
    INFO("shadowed tau " << tau_shadow);
    REQUIRE(tau_shadow > 3.0);
    for (const math::Vec3& p : {sunlit, shadowed}) {
        double mean = 0.0, mean_t = 0.0, lo = 1e9, hi = 0.0;
        for (int j = 0; j < 4096; ++j) {
            const double tau = marched(p, (j + 0.5) / 4096.0, march);
            mean += tau;
            mean_t += std::exp(-tau);
            lo = std::min(lo, tau);
            hi = std::max(hi, tau);
        }
        mean /= 4096.0;
        mean_t /= 4096.0;
        const double exact = exact_tau(p);
        report("Sun depth exact %.4f, 6-sample march mean %.4f (%.3f .. %.3f), T %.4f, mean T %.4f\n", exact,
                    mean, lo, hi, std::exp(-exact), mean_t);
        CHECK(mean == Approx(exact).epsilon(0.01));
        CHECK(std::exp(-mean) == Approx(std::exp(-exact)).epsilon(0.02));
    }
    // The geometric near intervals end at near_length.
    const double r = cloud_light_march_ratio(march);
    CHECK(march.first_m * (std::pow(r, 6) - 1.0) / (r - 1.0) == Approx(6000.0).epsilon(1e-9));

    // The source on a thick cumulus (in-cloud extinction 0.03 /m, tau 36 across) under a low Sun (10 deg) and a Sun
    // 3 deg below the local horizon (sunset), the camera on the Sun's side looking at the flank (view propagation
    // toward the camera = the horizontal direction to the Sun). Points 30 m inside the flank at the middle height.
    const math::Vec3 up = {0.0, 0.0, 1.0};
    const double thick = 0.03;
    auto thick_at = [&](const math::Vec3& p) { return thick * cloud_edge_ramp((radius - math::norm(p)) / edge); };
    auto integrate = [&](const math::Vec3& from, const math::Vec3& dir) {
        double tau = 0.0;
        const double ds = 0.05;
        for (double s = 0.5 * ds; s < 2.0 * radius; s += ds) tau += thick_at(from + dir * s) * ds;
        return tau;
    };
    for (double elevation_deg : {10.0, -3.0}) {
        const double e = elevation_deg * c::deg_to_rad;
        const math::Vec3 to_sun = {std::cos(e), 0.0, std::sin(e)};
        const math::Vec3 lit_point = math::Vec3{1.0, 0.0, 0.0} * (radius - 30.0);
        const math::Vec3 dark_point = math::Vec3{-1.0, 0.0, 0.0} * (radius - 30.0);
        const math::Vec3 view = {1.0, 0.0, 0.0};
        const double column = integrate(math::Vec3{0.0, 0.0, -radius}, up);
        auto source = [&](const math::Vec3& p) {
            return cloud_scattering_source_3d(to_sun, view, up, integrate(p, to_sun), integrate(p, up), column);
        };
        const double lit = source(lit_point), dark = source(dark_point);
        // The column model (the plane-parallel top incidence mu0, the Sun's depth from the vertical one).
        const double flat = cloud_scattering_source(-math::dot(to_sun, view), 0.0, std::sin(e),
                                                    integrate(lit_point, up) / std::max(std::sin(e), 1e-3),
                                                    integrate(lit_point, up), column);
        report("Sun %+.0f deg: source sunlit flank %.5f, shadowed %.5f, column model %.5f per sr\n",
                    elevation_deg, lit, dark, flat);
        CHECK(lit > 3.0 * dark);
        CHECK(lit > 2.0 * flat);  // the flank's true incidence, not mu0 E (and lit below the local horizon)
        CHECK(cloud_entry_cosine(std::sin(e), integrate(lit_point, to_sun), integrate(lit_point, up)) >
              std::max(std::sin(e), 0.0) + 0.4);  // measured 0.78 / 0.57
    }
    // A horizontally uniform slab: the 3D source is the column source (tau_s = t / mu0 gives mu_e = mu0).
    for (double t : {0.5, 2.0, 8.0}) {
        const double mu0 = 0.6;
        const math::Vec3 to_sun = {std::sqrt(1.0 - mu0 * mu0), 0.0, mu0};
        const math::Vec3 view = math::normalized(math::Vec3{-0.3, 0.2, 0.8});
        CHECK(cloud_entry_cosine(mu0, t / mu0, t) == Approx(mu0).epsilon(1e-12));
        CHECK(cloud_scattering_source_3d(to_sun, view, up, t / mu0, t, 10.0) ==
              Approx(cloud_scattering_source(-math::dot(to_sun, view), math::dot(view, up), mu0, t / mu0, t, 10.0))
                  .epsilon(1e-9));
    }
}

TEST_CASE("Sun transmittance through a uniform slab", "[weather]") {
    // One 1-km layer (1-2 km: four whole 250 m layers) of liquid water, everywhere. The snapshot's profile is linear
    // between layer centres, so its vertical integral is the slab's (equal layer thickness around it).
    const WeatherSnapshot slab = WeatherSnapshot::uniform_slab(19, 36, 1000.0, 2000.0, 2e-6, 1.0);
    const double latitude = 30.0 * c::deg_to_rad, longitude = 45.0 * c::deg_to_rad;
    const WeatherCell cell = slab.at(latitude, longitude, 1500.0);
    const double vertical_tau = 1000.0 * cloud_extinction_per_m(cell.liquid_water_kg_m3, cell.ice_water_kg_m3);
    REQUIRE(vertical_tau == Approx(0.3).epsilon(0.03));  // encoding error <= 2 %

    for (double zenith_deg : {0.0, 60.0}) {
        const double zenith = zenith_deg * c::deg_to_rad;
        const LocalFrame ground = local_frame(latitude, longitude, 0.0);
        const double below = cloud_sun_transmittance(slab, ground.position, sun_at_zenith_angle(ground, zenith));
        CHECK(below == Approx(std::exp(-vertical_tau / std::cos(zenith))).epsilon(0.01));
        // From inside the slab (its middle): half the path.
        const LocalFrame inside = local_frame(latitude, longitude, 1500.0);
        const double half = cloud_sun_transmittance(slab, inside.position, sun_at_zenith_angle(inside, zenith));
        CHECK(half == Approx(std::exp(-0.5 * vertical_tau / std::cos(zenith))).epsilon(0.01));
        // Above the slab nothing is in the way.
        const LocalFrame above = local_frame(latitude, longitude, 25000.0);
        CHECK(cloud_sun_transmittance(slab, above.position, sun_at_zenith_angle(above, zenith)) == 1.0);
    }
    // A Sun 5.5 deg below the horizon from 30 km: the ray dips to ~0.5 km and crosses the layer twice: finite, < 1.
    const LocalFrame high = local_frame(latitude, longitude, 30000.0);
    const double grazing =
        cloud_sun_transmittance(slab, high.position, sun_at_zenith_angle(high, 95.5 * c::deg_to_rad));
    CHECK(std::isfinite(grazing));
    CHECK(grazing < 1.0);

    // Sky factor: the two-stream diffuse transmittance of the column above.
    const LocalFrame ground = local_frame(latitude, longitude, 0.0);
    CHECK(cloud_sky_factor(slab, ground.position) ==
          Approx(two_stream_diffuse_transmittance(vertical_tau, cloud_asymmetry_g())).epsilon(1e-3));
    const LocalFrame inside = local_frame(latitude, longitude, 1500.0);
    CHECK(cloud_sky_factor(slab, inside.position) ==
          Approx(two_stream_diffuse_transmittance(0.5 * vertical_tau, cloud_asymmetry_g())).epsilon(1e-3));
    CHECK(cloud_sky_factor(slab, local_frame(latitude, longitude, 2500.0).position) == 1.0);

    // Poles and the date line (Review Focus 4): finite, the same value on both sides of the seam.
    for (double lat_deg : {90.0, -90.0, 0.0}) {
        const double lat = lat_deg * c::deg_to_rad;
        const LocalFrame east = local_frame(lat, c::pi - 1e-9, 0.0), west = local_frame(lat, -c::pi + 1e-9, 0.0);
        const double te = cloud_sun_transmittance(slab, east.position, sun_at_zenith_angle(east, 0.5));
        const double tw = cloud_sun_transmittance(slab, west.position, sun_at_zenith_angle(west, 0.5));
        CHECK(std::isfinite(te));
        CHECK(te == Approx(tw).epsilon(1e-6));
        CHECK(std::isfinite(cloud_sky_factor(slab, east.position)));
    }
}

TEST_CASE("The Sun's light diffused through a uniform slab", "[weather]") {
    // tau ~3 slab at 1-2 km: under it the Sun's diffuse part is the slab's total minus its direct transmittance
    // (delta-Eddington without the table), half the column inside, nothing above or with the Sun below the horizon.
    const WeatherSnapshot slab = WeatherSnapshot::uniform_slab(19, 36, 1000.0, 2000.0, 2e-5, 1.0);
    const double latitude = 30.0 * c::deg_to_rad, longitude = 45.0 * c::deg_to_rad;
    const LocalFrame ground = local_frame(latitude, longitude, 0.0);
    const double tau = cloud_column_optical_depth_above(slab, latitude, longitude, 0.0);
    REQUIRE(tau > 2.5);
    const double g = cloud_asymmetry_g();
    for (double zenith_deg : {0.0, 60.0}) {
        const double mu0 = std::cos(zenith_deg * c::deg_to_rad);
        const math::Vec3 sun = sun_at_zenith_angle(ground, zenith_deg * c::deg_to_rad);
        CHECK(cloud_sun_diffuse_transmittance(slab, nullptr, ground.position, sun) ==
              Approx(two_stream_beam_transmittance(tau, g, mu0) - std::exp(-tau / mu0)).epsilon(1e-3));
        const LocalFrame inside = local_frame(latitude, longitude, 1500.0);
        CHECK(cloud_sun_diffuse_transmittance(slab, nullptr, inside.position, sun) ==
              Approx(two_stream_beam_transmittance(0.5 * tau, g, mu0) - std::exp(-0.5 * tau / mu0)).epsilon(1e-2));
        CHECK(cloud_sun_diffuse_transmittance(slab, nullptr, local_frame(latitude, longitude, 2500.0).position, sun) ==
              0.0);
    }
    CHECK(cloud_sun_diffuse_transmittance(slab, nullptr, ground.position, sun_at_zenith_angle(ground, 1.7)) == 0.0);
    CHECK(two_stream_beam_transmittance(0.0, g, 0.5) == Approx(1.0));
}

TEST_CASE("Broken cloud lights the ground as independent columns", "[weather]") {
    // A cumulus layer of cloud fraction 0.4 (in-cloud tau ~7.5 over 1 km): the clear part of the beam passes the Sun, the
    // beam's cover C_b (cloud_beam_cover) passes exp(-tau_grid / (C_b mu0)) - not exp(-grid-mean tau / mu0) (the
    // plane-parallel bias); the Sun's diffuse light and the sky likewise (the sky with the column's maximum-overlap cover).
    const WeatherSnapshot slab = WeatherSnapshot::uniform_slab(19, 36, 1000.0, 2000.0, 2e-5, 0.4);
    const double latitude = 30.0 * c::deg_to_rad, longitude = 45.0 * c::deg_to_rad;
    const LocalFrame ground = local_frame(latitude, longitude, 0.0);
    const CloudColumnDepths column = cloud_column_depths_above(slab, latitude, longitude, 0.0);
    REQUIRE(column.cover() == Approx(0.4).epsilon(0.02));
    const double tau_in = column.in_cloud_depth();
    REQUIRE(tau_in == Approx(column.mean / 0.4).epsilon(0.02));
    const double g = cloud_asymmetry_g();
    const double cover = column.cover();
    for (double zenith_deg : {0.0, 60.0}) {
        const double mu0 = std::cos(zenith_deg * c::deg_to_rad);
        const math::Vec3 sun = sun_at_zenith_angle(ground, zenith_deg * c::deg_to_rad);
        const double beam = cloud_beam_cover(column, mu0);
        CHECK(beam > cover);  // the shapes' shadows cover more than the cloud fraction
        const double tau_beam = column.mean / beam;
        CHECK(cloud_sun_transmittance(slab, ground.position, sun) ==
              Approx(1.0 - beam + beam * std::exp(-tau_beam / mu0)).epsilon(0.01));
        CHECK(cloud_sun_diffuse_transmittance(slab, nullptr, ground.position, sun) ==
              Approx(beam * (two_stream_beam_transmittance(tau_beam, g, mu0) - std::exp(-tau_beam / mu0)))
                  .epsilon(1e-3));
    }
    CHECK(cloud_sky_factor(slab, ground.position) ==
          Approx(1.0 - cover + cover * two_stream_diffuse_transmittance(tau_in, g)).epsilon(1e-3));
}

TEST_CASE("The beam's cover follows the fitted overlap of the shapes' shadows", "[weather]") {
    // cloud_beam_cover = 1 - (1 - f)^n with n = 1 + 0.66 (1 - f) + 0.21 max(cot e - 1.8, 0): more than f even at the
    // zenith, growing as the Sun sinks, 1 for a deck, 0 without cloud, bounded below the horizon by mu0 >= 0.02.
    CHECK(cloud_beam_cover(1.0, 0.5) == 1.0);
    CHECK(cloud_beam_cover(CloudColumnDepths{}, 0.5) == 0.0);
    const double f = 0.3;
    CHECK(cloud_beam_cover(f, 1.0) == Approx(1.0 - std::pow(1.0 - f, 1.0 + 0.66 * (1.0 - f))).epsilon(1e-12));
    double previous = f;
    for (double elevation_deg : {90.0, 60.0, 30.0, 15.0, 8.0, 3.0, 0.5}) {
        const double cover = cloud_beam_cover(f, std::sin(elevation_deg * c::deg_to_rad));
        CHECK(cover >= previous - 1e-12);
        CHECK(cover <= 1.0);
        previous = cover;
    }
    // Shape fit (the shipped shapes' shadows, cloud_check.gd section 7): cf 0.3 at 8 deg: the shadows of the shapes
    // leave 0.404 of the Sun (tau_in 3: no light through the cloud), the model 1 - C within 0.02.
    CHECK(1.0 - cloud_beam_cover(0.3, std::sin(8.0 * c::deg_to_rad)) == Approx(0.404).margin(0.03));
    CHECK(1.0 - cloud_beam_cover(0.4, std::sin(45.0 * c::deg_to_rad)) == Approx(0.517).margin(0.03));
}

TEST_CASE("Mid-blend light follows the renderer's order (blend the column, then the transmittance)", "[weather]") {
    // A clear snapshot blended with a cf 0.4 one at w = 0.5: the renderer's column is half the depth and half the
    // cover, and the transmittances come from that column - not the mean of the two snapshots' transmittances.
    const WeatherSnapshot broken = WeatherSnapshot::uniform_slab(19, 36, 1000.0, 2000.0, 2e-5, 0.4);
    const WeatherSnapshot clear = WeatherSnapshot::uniform_slab(19, 36, 1000.0, 2000.0, 0.0, 0.0);
    const double latitude = 30.0 * c::deg_to_rad, longitude = 45.0 * c::deg_to_rad;
    const LocalFrame ground = local_frame(latitude, longitude, 0.0);
    const math::Vec3 sun = sun_at_zenith_angle(ground, 60.0 * c::deg_to_rad);
    const double mu0 = 0.5;
    const CloudColumnDepths b = cloud_sun_ray_depths(broken, ground.position, sun);
    const CloudColumnDepths k = cloud_sun_ray_depths(clear, ground.position, sun);
    const CloudColumnDepths half = blend_cloud_column_depths(k, b, 0.5);
    CHECK(half.mean == Approx(0.5 * b.mean));
    CHECK(half.max_fraction == Approx(0.5 * b.max_fraction));
    const double beam = cloud_beam_cover(half, mu0);
    CHECK(cloud_sun_transmittance(half, mu0) ==
          Approx(1.0 - beam + beam * std::exp(-half.mean / beam)).epsilon(1e-9));
    CHECK(std::abs(cloud_sun_transmittance(half, mu0) -
                   0.5 * (cloud_sun_transmittance(b, mu0) + cloud_sun_transmittance(k, mu0))) > 0.005);
    const double cover = half.cover();
    CHECK(cover == Approx(0.2).epsilon(0.02));
    CHECK(cloud_sky_factor(half) ==
          Approx(1.0 - cover + cover * two_stream_diffuse_transmittance(half.in_cloud_depth(), cloud_asymmetry_g()))
              .epsilon(1e-9));
    CHECK(cloud_sun_diffuse_transmittance(half, nullptr, mu0) ==
          Approx(beam * (two_stream_beam_transmittance(half.mean / beam, cloud_asymmetry_g(), mu0) -
                         std::exp(-half.mean / (beam * mu0))))
              .epsilon(1e-9));
    // Broken cloud blended into a thin deck: depth mixes linearly, the cover as the linear mix of the fractions.
    const WeatherSnapshot deck = WeatherSnapshot::uniform_slab(19, 36, 1000.0, 2000.0, 2e-6, 1.0);
    const CloudColumnDepths d = cloud_sun_ray_depths(deck, ground.position, sun);
    const CloudColumnDepths mix = blend_cloud_column_depths(b, d, 0.25);
    CHECK(mix.mean == Approx(0.75 * b.mean + 0.25 * d.mean));
    CHECK(mix.max_fraction == Approx(0.75 * b.max_fraction + 0.25 * d.max_fraction));
    const double mean_t = 0.75 * cloud_sun_transmittance(b, mu0) + 0.25 * cloud_sun_transmittance(d, mu0);
    CHECK(std::abs(cloud_sun_transmittance(mix, mu0) - mean_t) > 0.01);
}
