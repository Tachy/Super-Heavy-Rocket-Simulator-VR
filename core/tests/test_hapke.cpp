// Hapke reflectance (Task 9 of the Moon procedural surface plan): published check values of the implementation
// (USGS ISIS photometric unit-test truths, Chandrasekhar's H), reciprocity, the disc integrator against the analytic
// Lommel-Seeliger sphere, the disc-integrated phase curve of the shipped parameter set against ROLO (Kieffer & Stone
// 2005) and the full-Moon disc against the game's former Lommel-Seeliger globe.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "starsim/config/moon_surface_config.hpp"
#include "starsim/environment/hapke.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using environment::HapkeHFunction;

namespace {

constexpr double deg = constants::deg_to_rad;

config::HapkeParameters shipped_hapke() {
    return config::load_moon_surface_config(std::filesystem::path(STARSIM_CONFIG_DIR) / "moon_surface.toml").hapke;
}

double relative_error(double got, double expected) { return std::abs(got / expected - 1.0); }

// USGS ISIS3 HapkeHen / HapkeLeg photometric models (isis/src/base/objs/Hapke: Hapke.cpp, unitTest.cpp,
// Hapke.truth; public domain, CC0): the 1981 H function, the double Henyey-Greenstein pg = (1 - Hg2) HG_forward(Hg1)
// + Hg2 HG_back(Hg1), i.e. b = Hg1, c = 2 Hg2 - 1, K = 1, no CBOE. Albedo = I/F.
struct IsisCase {
    config::HapkeParameters p;
    double theta_deg;
    double phase_deg, incidence_deg, emission_deg, truth;
    const char* what;
};

config::HapkeParameters isis_parameters(double wh, double b0, double hh, double hg1, double hg2) {
    config::HapkeParameters p;
    p.w = wh;
    p.b = hg1;
    p.c = 2.0 * hg2 - 1.0;
    p.bs0 = b0;
    p.hs = hh;
    p.bc0 = 0.0;
    p.hc = 1.0;
    p.porosity_k = 1.0;
    return p;
}

// ROLO disc-equivalent reflectance A_k (Kieffer & Stone 2005, AJ 129, 2887, eq. 10 / Table 4) at 665.1 nm, zero
// libration, mean of the waxing (Phi = +g) and waning (Phi = -g) Moon. Coefficients as transcribed by J. Hollan
// (amper.ped.muni.cz/light/luminance/moon_cal/r554.gnp, "kindly sent by Tom Stone"; c and p are the paper's
// wavelength-independent values). g in rad for the polynomial, in degrees in the exponential / cosine terms (p1..p4 in
// degrees). Valid 1.55-97 deg.
double rolo_665(double phase_deg) {
    const double g = phase_deg * deg;
    const double a[4] = {-1.88914, -1.58096, 0.30477, -0.17908};
    const double b[3] = {0.04415, 0.00983, -0.00389};
    const double d[3] = {0.37141, -0.13514, 0.01248};
    const double p[4] = {4.06054, 12.8802, -30.5858, 16.7498};
    double mean = 0.0;
    for (double sign : {1.0, -1.0}) {
        const double phi = sign * g;
        const double ln_a = a[0] + a[1] * g + a[2] * g * g + a[3] * g * g * g + b[0] * phi +
                            b[1] * phi * phi * phi + b[2] * std::pow(phi, 5) + d[0] * std::exp(-phase_deg / p[0]) +
                            d[1] * std::exp(-phase_deg / p[1]) + d[2] * std::cos((phase_deg - p[2]) / p[3]);
        mean += 0.5 * std::exp(ln_a);
    }
    return mean;
}

// The game's former Moon shading (game/shaders/moon_regolith.gdshaderinc, regolith_reflectance with moon_brdf =
// lommel_seeliger): Lommel-Seeliger 2 mu0 / (mu0 + mu) x f(g), f = K&S 1991 Eq. 9 x the opposition surge / Phi_LS.
double ks_relative(double phase_deg) {
    const double surge = phase_deg < 7.0 ? 1.35 - 0.05 * phase_deg : 1.0;
    return surge * std::pow(10.0, -0.4 * (0.026 * phase_deg + 4e-9 * std::pow(phase_deg, 4)));
}

double ls_sphere_phase(double phase_rad) {
    if (phase_rad < 1e-6) return 1.0;
    return 1.0 - std::sin(0.5 * phase_rad) * std::tan(0.5 * phase_rad) * std::log(1.0 / std::tan(0.25 * phase_rad));
}

double old_globe_disc(double phase_deg) {
    const double g = phase_deg * deg;
    const double f = ks_relative(phase_deg) / ls_sphere_phase(g);
    return environment::disc_integrated_reflectance(
        [&](double mu0, double mu) { return 2.0 * mu0 / (mu0 + mu) * f; }, g);
}

// The game's normalisation (Atmosphere.hapke_normalisation): the Hapke disc at full phase equals the former globe's.
double game_normalisation(const config::HapkeParameters& p) {
    return 1.35 / environment::hapke_disc_reflectance(p, 0.0, p.theta_bar_rad);
}

// Exact H by iterating 1/H(x) = sqrt(1 - w) + w/2 int_0^1 x' H(x') / (x + x') dx' (Chandrasekhar 1960) on a midpoint
// rule in u = sqrt(x') (nodes dense near 0); evaluated at x by one more pass of the same equation.
double exact_h(double x, double w) {
    constexpr int n = 400;
    std::vector<double> nodes(n), weights(n), h(n, 1.0);
    for (int k = 0; k < n; ++k) {
        const double u = (k + 0.5) / n;
        nodes[k] = u * u;
        weights[k] = 2.0 * u / n;  // dx' = 2u du
    }
    for (int iteration = 0; iteration < 500; ++iteration) {
        double change = 0.0;
        for (int i = 0; i < n; ++i) {
            double integral = 0.0;
            for (int k = 0; k < n; ++k) integral += weights[k] * nodes[k] * h[k] / (nodes[i] + nodes[k]);
            const double next = 1.0 / (std::sqrt(1.0 - w) + 0.5 * w * integral);
            change = std::max(change, std::abs(next - h[i]));
            h[i] = next;
        }
        if (change < 1e-12) break;
    }
    double integral = 0.0;
    for (int k = 0; k < n; ++k) integral += weights[k] * nodes[k] * h[k] / (x + nodes[k]);
    return 1.0 / (std::sqrt(1.0 - w) + 0.5 * w * integral);
}

}  // namespace

TEST_CASE("Hapke: USGS ISIS photometric unit-test truths", "[moon_surface][hapke]") {
    const auto defaults = isis_parameters(0.5, 0.0, 0.0, 0.0, 0.0);  // HapkeHen defaults: smooth, isotropic, no surge
    const auto hen = isis_parameters(0.52, 1.0, 1.0, 0.213, 1.0);
    const auto leg = isis_parameters(0.52, 1.0, 1.0, 0.0, 0.5);  // HapkeLeg with Bh = Ch = 0: p = 1
    const IsisCase cases[] = {
        {defaults, 0.0, 0.0, 0.0, 0.0, 0.0965097, "HapkeHen defaults (0, 0, 0)"},
        {defaults, 0.0, 60.0, 45.0, 30.0, 0.0832883, "HapkeHen defaults (60, 45, 30)"},
        {hen, 30.0, 0.0, 0.0, 0.0, 0.286048, "HapkeHen w .52 B0 1 h 1 theta 30 Hg1 .213 Hg2 1 (0, 0, 0)"},
        {hen, 30.0, 60.0, 45.0, 30.0, 0.1342, "HapkeHen w .52 B0 1 h 1 theta 30 Hg1 .213 Hg2 1 (60, 45, 30)"},
        {leg, 30.0, 0.0, 0.0, 0.0, 0.16145, "HapkeLeg w .52 B0 1 h 1 theta 30 (0, 0, 0)"},
        {leg, 30.0, 60.0, 45.0, 30.0, 0.111525, "HapkeLeg w .52 B0 1 h 1 theta 30 (60, 45, 30)"},
    };
    for (const auto& c : cases) {
        const double got = environment::hapke_reflectance(c.p, std::cos(c.incidence_deg * deg),
                                                          std::cos(c.emission_deg * deg), c.phase_deg * deg,
                                                          c.theta_deg * deg, HapkeHFunction::hapke_1981);
        std::printf("  ISIS %-62s truth %.6f got %.6f (%+.4f %%)\n", c.what, c.truth, got,
                    100.0 * (got / c.truth - 1.0));
        INFO(c.what << ": truth " << c.truth << " got " << got);
        CHECK(relative_error(got, c.truth) < 0.01);  // the brief's 1 %
    }
    // (180, 90, 90): the light at the horizon gives 0.
    CHECK(environment::hapke_reflectance(hen, 0.0, 0.0, constants::pi, 30.0 * deg, HapkeHFunction::hapke_1981) == 0.0);
}

TEST_CASE("Hapke: H function against Chandrasekhar's exact H", "[moon_surface][hapke]") {
    // [S] Chandrasekhar 1960 (Radiative Transfer, H-function tables): conservative isotropic scattering, H(1) = 2.9078.
    const double h1 = environment::hapke_h_function(1.0, 1.0 - 1e-12, HapkeHFunction::hapke_2002);
    std::printf("  H(1, w = 1): Hapke 2002 %.4f vs Chandrasekhar 2.9078 (%+.2f %%)\n", h1, 100.0 * (h1 / 2.9078 - 1));
    CHECK(relative_error(h1, 2.9078) < 0.01);
    // The shipped w against the exact integral equation (Hapke 2002: < 1 % for all w).
    const double w = shipped_hapke().w;
    for (double x : {0.05, 0.1, 0.25, 0.5, 0.75, 1.0}) {
        const double exact = exact_h(x, w);
        const double approx = environment::hapke_h_function(x, w, HapkeHFunction::hapke_2002);
        INFO("x " << x << " exact " << exact << " Hapke 2002 " << approx);
        CHECK(relative_error(approx, exact) < 0.01);
    }
    // The solver itself against the published conservative value (w -> 1 converges slowly: 0.9999 within 2 %).
    CHECK(relative_error(exact_h(1.0, 0.9999), 2.9078) < 0.02);
}

TEST_CASE("Hapke: reciprocity, smooth limit and the horizon", "[moon_surface][hapke]") {
    const auto p = shipped_hapke();
    for (const auto& [i, e, g] : {std::array<double, 3>{30, 60, 50}, {10, 70, 75}, {60, 20, 45}, {80, 5, 80}}) {
        const double mu0 = std::cos(i * deg), mu = std::cos(e * deg);
        const double forward = environment::hapke_reflectance(p, mu0, mu, g * deg, p.theta_bar_rad) / mu0;
        const double reverse = environment::hapke_reflectance(p, mu, mu0, g * deg, p.theta_bar_rad) / mu;
        INFO(i << " " << e << " " << g);
        CHECK(std::abs(forward / reverse - 1.0) < 1e-12);  // Helmholtz reciprocity of the BRDF I/F / mu0
    }
    // theta_bar = 0: the smooth IMSA closed form.
    const double mu0 = std::cos(40 * deg), mu = std::cos(25 * deg), g = 55 * deg;
    const double hs = 1.0 / (1.0 + std::tan(0.5 * g) / p.hs);
    const double smooth = 0.25 * p.w * mu0 / (mu0 + mu) *
                          (environment::hapke_phase_function(p.b, p.c, g) * (1.0 + p.bs0 * hs) +
                           environment::hapke_h_function(mu0, p.w, HapkeHFunction::hapke_2002) *
                               environment::hapke_h_function(mu, p.w, HapkeHFunction::hapke_2002) -
                           1.0);
    CHECK(relative_error(environment::hapke_reflectance(p, mu0, mu, g, 0.0), smooth) < 1e-12);
    CHECK(environment::hapke_reflectance(p, 0.0, 0.5, 0.3, p.theta_bar_rad) == 0.0);
    CHECK(environment::hapke_reflectance(p, 0.5, -0.1, 0.3, p.theta_bar_rad) == 0.0);
    // Grazing view on a lit facet (the crest-line case): Lommel-Seeliger doubles from mu = mu0 to mu -> 0; the rough
    // Hapke surface brightens far less (its effective cosine stays > 0: the facets tilted toward the eye).
    const double grazing = environment::hapke_reflectance(p, 0.5, 1e-3, 60 * deg, p.theta_bar_rad);
    const double equal = environment::hapke_reflectance(p, 0.5, 0.5, 60 * deg, p.theta_bar_rad);
    std::printf("  grazing (mu0 0.5, mu 1e-3) / (mu 0.5): Hapke %.3f, Lommel-Seeliger %.3f\n", grazing / equal,
                2.0 * 0.5 / (0.5 + 1e-3));
    CHECK(std::isfinite(grazing));
    CHECK(grazing / equal < 1.5);
}

TEST_CASE("Hapke: disc integrator against the analytic Lommel-Seeliger sphere", "[moon_surface][hapke]") {
    for (double g_deg : {0.0, 5.0, 30.0, 60.0, 90.0, 120.0, 150.0}) {
        const double g = g_deg * deg;
        const double disc = environment::disc_integrated_reflectance(
            [](double mu0, double mu) { return 2.0 * mu0 / (mu0 + mu); }, g);
        INFO("g " << g_deg << " disc " << disc << " analytic " << ls_sphere_phase(g));
        CHECK(std::abs(disc - ls_sphere_phase(g)) < 1e-5);
    }
    const auto p = shipped_hapke();
    const double coarse = environment::hapke_disc_reflectance(p, 40 * deg, p.theta_bar_rad, 48);
    const double fine = environment::hapke_disc_reflectance(p, 40 * deg, p.theta_bar_rad, 96);
    CHECK(relative_error(coarse, fine) < 1e-5);
}

TEST_CASE("Hapke: disc-integrated phase curve against ROLO 665 nm", "[moon_surface][hapke]") {
    const auto p = shipped_hapke();
    // Phase 5 deg (the plan) to 97 deg (ROLO's upper validity limit), 1 deg steps. Magnitude residual
    // m_Hapke - m_ROLO; its mean is the level difference (the shipped highland-proxy w against ROLO's whole near side at
    // 665 nm), the spread about it is the shape error the plan bounds at 0.1 mag.
    std::vector<double> residual;
    for (int g = 5; g <= 97; ++g) {
        const double hapke = environment::hapke_disc_reflectance(p, g * deg, p.theta_bar_rad);
        residual.push_back(-2.5 * std::log10(hapke / rolo_665(g)));
    }
    double mean = 0.0;
    for (double r : residual) mean += r / static_cast<double>(residual.size());
    double worst = 0.0;
    int worst_g = 0;
    for (std::size_t k = 0; k < residual.size(); ++k) {
        if (std::abs(residual[k] - mean) > worst) {
            worst = std::abs(residual[k] - mean);
            worst_g = 5 + static_cast<int>(k);
        }
    }
    std::printf("  ROLO 665 nm, 5-97 deg: mean m_Hapke - m_ROLO %+.3f mag, worst shape residual %.3f mag at %d deg\n",
                mean, worst, worst_g);
    for (int g : {5, 10, 20, 30, 45, 60, 75, 90, 97})
        std::printf("    g %3d  A_Hapke %.5f  A_ROLO %.5f  residual %+.3f\n", g,
                    environment::hapke_disc_reflectance(p, g * deg, p.theta_bar_rad), rolo_665(g),
                    residual[static_cast<std::size_t>(g - 5)] - mean);
    CHECK(worst < 0.1);
    // Informative: the K&S 1991 Eq. 9 law of the former shading, 5-120 deg, same statistic (not a gate: K&S is the
    // older V-band summary; ROLO is the modern reference and stops at 97 deg).
    std::vector<double> ks;
    for (int g = 5; g <= 120; ++g)
        ks.push_back(-2.5 * std::log10(environment::hapke_disc_reflectance(p, g * deg, p.theta_bar_rad) /
                                       ks_relative(g)));
    double ks_mean = 0.0, ks_worst = 0.0;
    for (double r : ks) ks_mean += r / static_cast<double>(ks.size());
    for (double r : ks) ks_worst = std::max(ks_worst, std::abs(r - ks_mean));
    std::printf("  K&S 1991 Eq. 9, 5-120 deg: worst shape residual %.3f mag (informative)\n", ks_worst);
}

TEST_CASE("Hapke: normalisation identity at phase 0 and the disc at realistic full-Moon phases",
          "[moon_surface][hapke]") {
    const auto p = shipped_hapke();
    const double norm = game_normalisation(p);
    std::printf("  normalisation 1.35 / A_Hapke(0) = %.5f (A_Hapke(0) = %.5f)\n", norm,
                environment::hapke_disc_reflectance(p, 0.0, p.theta_bar_rad));
    for (double g : {0.0, 0.5, 1.0, 1.5, 2.0, 3.0, 5.0, 7.0, 10.0}) {
        const double hapke = norm * environment::hapke_disc_reflectance(p, g * deg, p.theta_bar_rad, 96);
        const double old_globe = old_globe_disc(g);
        std::printf("    g %4.1f  Hapke x norm %.5f  former globe %.5f  ratio %.4f  ROLO(g)/ROLO(0) %.4f%s\n", g, hapke,
                    old_globe, hapke / old_globe, rolo_665(g) / rolo_665(0.0), g < 1.55 ? " (ROLO extrapolated)" : "");
    }
    // NORMALISATION IDENTITY, not an independent physics check (Ruling T9 I1): the gain is defined as 1.35 / A_Hapke(0),
    // the former globe's full-phase disc (the regolith scale's calibration point, Atmosphere.regolith_scale). The
    // discs are integrated independently (order 96 against the normalisation's 48; the former globe through the
    // Lommel-Seeliger integrator), so this checks the integrator and the wiring, not Hapke.
    const double full_hapke = norm * environment::hapke_disc_reflectance(p, 0.0, p.theta_bar_rad, 96);
    CHECK(relative_error(full_hapke, old_globe_disc(0.0)) < 0.03);
    // A real full Moon outside an eclipse is at g = 1.5-5 deg: Hapke is 0.96-1.0 of the former law (3.8 % low at 2 deg).
    // Accepted by Ruling T9 I1: the difference moves toward ROLO (disc(2)/disc(0): Hapke 0.849, K&S 0.883, ROLO 0.834,
    // the latter extrapolated below 1.55 deg), the physics gate is the ROLO phase curve above. Gate 5 %.
    for (double g : {1.5, 2.0, 3.0, 5.0}) {
        const double hapke = norm * environment::hapke_disc_reflectance(p, g * deg, p.theta_bar_rad, 96);
        INFO("g " << g);
        CHECK(relative_error(hapke, old_globe_disc(g)) < 0.05);
    }
}

// CBOE (Hapke 2012): r x (1 + B_C0 B_C(g)), B_C = (1 + (1 - e^-z)/z) / (2 (1 + z)^2), z = tan(g/2) / h_C. Not in the
// shipped set (B_C0 = 0) and not in the ISIS truths; the reference below is written out independently of hapke.cpp.
TEST_CASE("Hapke: coherent-backscatter opposition effect (CBOE)", "[moon_surface][hapke]") {
    auto p = shipped_hapke();
    auto with_cboe = p;
    with_cboe.bc0 = 0.6;
    with_cboe.hc = 0.05;
    const double mu0 = std::cos(35 * deg), mu = std::cos(20 * deg);
    for (double g_deg : {0.5, 2.0, 5.0, 20.0, 60.0}) {
        const double g = g_deg * deg;
        const double z = std::tan(0.5 * g) / with_cboe.hc;
        const double b_c = (1.0 + (1.0 - std::exp(-z)) / z) / (2.0 * (1.0 + z) * (1.0 + z));
        const double base = environment::hapke_reflectance(p, mu0, mu, g, p.theta_bar_rad);
        const double boosted = environment::hapke_reflectance(with_cboe, mu0, mu, g, p.theta_bar_rad);
        INFO("g " << g_deg);
        CHECK(relative_error(boosted / base, 1.0 + with_cboe.bc0 * b_c) < 1e-12);
    }
    // B_C(0) = 1: the full factor 1 + B_C0 at zero phase; and decreasing toward large phase.
    const double at_zero = environment::hapke_reflectance(with_cboe, 1.0, 1.0, 0.0, 0.0) /
                           environment::hapke_reflectance(p, 1.0, 1.0, 0.0, 0.0);
    CHECK(relative_error(at_zero, 1.0 + with_cboe.bc0) < 1e-9);
    CHECK(environment::hapke_reflectance(with_cboe, mu0, mu, 1.0, 0.0) / environment::hapke_reflectance(p, mu0, mu, 1.0, 0.0) <
          1.0 + 0.01 * with_cboe.bc0);
    // B_C0 = 0 or h_C = 0 switch the effect off.
    with_cboe.bc0 = 0.0;
    CHECK(environment::hapke_reflectance(with_cboe, mu0, mu, 0.1, p.theta_bar_rad) ==
          environment::hapke_reflectance(p, mu0, mu, 0.1, p.theta_bar_rad));
}

// Porosity K (Hapke 2012 eq. 8.x): r = K w/4 mu0/(mu0 + mu) [p (1 + B) + H(mu0/K) H(mu/K) - 1] (smooth surface).
TEST_CASE("Hapke: porosity coefficient K", "[moon_surface][hapke]") {
    auto p = shipped_hapke();
    p.porosity_k = 1.2;
    const double mu0 = std::cos(40 * deg), mu = std::cos(25 * deg), g = 55 * deg;
    const double hs = 1.0 / (1.0 + std::tan(0.5 * g) / p.hs);
    const auto h = [&](double x) { return environment::hapke_h_function(x, p.w, HapkeHFunction::hapke_2002); };
    const double expected = p.porosity_k * 0.25 * p.w * mu0 / (mu0 + mu) *
                            (environment::hapke_phase_function(p.b, p.c, g) * (1.0 + p.bs0 * hs) +
                             h(mu0 / p.porosity_k) * h(mu / p.porosity_k) - 1.0);
    CHECK(relative_error(environment::hapke_reflectance(p, mu0, mu, g, 0.0), expected) < 1e-12);
    // The single-scattering limit (w -> 0): linear in K.
    auto dark = p;
    dark.w = 1e-6;
    auto dark_k1 = dark;
    dark_k1.porosity_k = 1.0;
    CHECK(relative_error(environment::hapke_reflectance(dark, mu0, mu, g, 0.0) /
                             environment::hapke_reflectance(dark_k1, mu0, mu, g, 0.0),
                         p.porosity_k) < 1e-4);
    // K differs from 1 measurably at the shipped albedo (the path is live), and K = 1 reproduces the plain IMSA.
    auto plain = p;
    plain.porosity_k = 1.0;
    CHECK(relative_error(environment::hapke_reflectance(p, mu0, mu, g, 0.0),
                         environment::hapke_reflectance(plain, mu0, mu, g, 0.0)) > 0.01);
    // Rough surface with K: reciprocity still holds.
    CHECK(std::abs(environment::hapke_reflectance(p, mu0, mu, g, p.theta_bar_rad) / mu0 /
                       (environment::hapke_reflectance(p, mu, mu0, g, p.theta_bar_rad) / mu) -
                   1.0) < 1e-12);
}
