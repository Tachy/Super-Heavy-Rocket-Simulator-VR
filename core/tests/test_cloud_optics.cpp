// Cloud optics, coverage-preserving noise and the CPU cloud light (M8 weather and clouds, Task 4).

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
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
    // Model: exact single scattering + the delta-Eddington diffuse field of the local column, integrated along view
    // rays; it is not flux-consistent with the two-stream fluxes. Tolerances are the measured errors against
    // delta-Eddington (task 4 report, Ruling 1): model is -8..-17 % for tau >= 10 and -25..-34 % for tau 1 and 3
    // (too dark), so tau >= 10 within 22 % and tau 1 / 3 within 36 %. The reference itself was checked with
    // tools/validation/cloud_slab_monte_carlo.py: delta-Eddington is within 7 % (mu0 = 1) / 10 % (mu0 = 0.5) of the
    // Monte Carlo truth, within 1 % for tau >= 30. Model constants are not tuned. Follow-up if thin cloud edges
    // render too dark: a Monte-Carlo transfer table (Bouthors et al. 2008).
    const double g = cloud_asymmetry_g();
    const bool verbose = std::getenv("STARSIM_VERBOSE") != nullptr;
    for (double zenith_deg : {0.0, 60.0}) {
        const double mu0 = std::cos(zenith_deg * c::deg_to_rad);
        for (double tau : {1.0, 3.0, 10.0, 30.0, 50.0}) {
            const double model = slab_reflectance(tau, mu0);
            const double reference = two_stream_beam_reflectance(tau, g, mu0);
            if (verbose) {
                std::printf("zenith %4.0f tau %4.0f model %.4f two-stream %.4f (%+.1f %%)\n", zenith_deg, tau, model,
                            reference, 100.0 * (model / reference - 1.0));
            }
            CHECK(model == Approx(reference).epsilon(tau >= 10.0 ? 0.22 : 0.36));
        }
    }
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
    CHECK(two_stream_reflectance(30.0, cloud_asymmetry_g()) + t == Approx(1.0).margin(1e-12));
}

TEST_CASE("Noise coverage is preserved", "[weather]") {
    const auto start = std::chrono::steady_clock::now();
    const CloudNoise noise = generate_cloud_noise();
    const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    std::printf("cloud noise generated in %.2f s\n", seconds);

    const int n = noise.shape_size;
    REQUIRE(noise.shape_rgba.size() == static_cast<std::size_t>(n) * n * n * 4);
    REQUIRE(noise.detail_rgba.size() ==
            static_cast<std::size_t>(noise.detail_size) * noise.detail_size * noise.detail_size * 4);
    const std::size_t texels = static_cast<std::size_t>(n) * n * n;
    for (double cf : {0.1, 0.5, 0.9}) {
        const float t = noise.coverage_threshold[static_cast<std::size_t>(std::lround(cf * 255.0))];
        std::size_t above = 0;
        for (std::size_t i = 0; i < texels; ++i) above += noise.shape_rgba[4 * i] / 255.0f > t ? 1 : 0;
        CHECK(static_cast<double>(above) / texels == Approx(cf).margin(0.02));
    }
    CHECK(noise.coverage_threshold[0] >= 1.0f);
    CHECK(noise.coverage_threshold[255] < 0.0f);
    for (int i = 1; i < 256; ++i) CHECK(noise.coverage_threshold[i] <= noise.coverage_threshold[i - 1]);

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
