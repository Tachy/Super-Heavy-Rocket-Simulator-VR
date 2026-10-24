// Drone-ship deck motion: the waterplane plane fit of each wave component, the heave / roll / pitch response, its
// statistics and the deck frame.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <complex>
#include <cstdio>

#include "starsim/config/drone_ship_config.hpp"
#include "starsim/contact/deck_motion.hpp"
#include "starsim/environment/sea_state.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using Catch::Approx;

namespace {

constexpr double g = constants::standard_gravity_mps2;

config::DroneShipHull test_hull() {
    config::DroneShipHull h;
    h.length_m = 91.4;
    h.beam_m = 30.5;
    h.draft_m = 3.0;
    h.roll_gyration_m = 0.35 * 30.5;
    h.pitch_gyration_m = 0.25 * 91.4;
    h.heave_added_mass_factor = 2.5;
    h.roll_added_inertia_factor = 0.5;
    h.pitch_added_inertia_factor = 3.3;
    h.heave_damping_ratio = 0.15;
    h.roll_damping_ratio = 0.10;
    h.pitch_damping_ratio = 0.15;
    return h;
}

double heave_omega(const config::DroneShipHull& h) {
    return std::sqrt(g / ((1.0 + h.heave_added_mass_factor) * h.draft_m));
}

// Box barge, KG at the waterline: GM = KB + BM - KG = d/2 + w^2/(12 d) - d.
double metacentric_height(double width_m, double draft_m) {
    return width_m * width_m / (12.0 * draft_m) - 0.5 * draft_m;
}

// Least-squares plane over the waterplane by an n x n midpoint sum: complex (mean, slope along the bow, slope to port)
// of exp(i k (x cos a - y sin a)) per unit amplitude.
contact::DeckResponse midpoint_plane(const config::DroneShipHull& h, double k, double a, int n) {
    const double dx = h.length_m / n, dy = h.beam_m / n;
    std::complex<double> sum_1, sum_x, sum_y;
    double sum_xx = 0.0, sum_yy = 0.0, count = 0.0;
    for (int i = 0; i < n; ++i) {
        const double x = -0.5 * h.length_m + (i + 0.5) * dx;
        for (int j = 0; j < n; ++j) {
            const double y = -0.5 * h.beam_m + (j + 0.5) * dy;
            const std::complex<double> e = std::polar(1.0, k * (x * std::cos(a) - y * std::sin(a)));
            sum_1 += e;
            sum_x += e * x;
            sum_y += e * y;
            sum_xx += x * x;
            sum_yy += y * y;
            count += 1.0;
        }
    }
    // x, y and 1 are orthogonal over the symmetric grid: the normal equations decouple
    return {sum_1 / count, sum_y / sum_yy, sum_x / sum_xx};  // heave, roll (port slope), pitch (bow slope)
}

}  // namespace

TEST_CASE("Long waves lift and tilt the deck with the sea", "[deck_motion]") {
    const auto h = test_hull();
    const double w = 0.05 * heave_omega(h);
    const double k = w * w / g;
    const auto head = contact::deck_response(h, w, 0.0);
    CHECK(std::abs(head.heave_m) == Approx(1.0).epsilon(0.02));
    CHECK(std::abs(head.pitch_rad) == Approx(k).epsilon(0.02));
    CHECK(std::abs(head.roll_rad) < 1e-6 * k);
    const auto beam = contact::deck_response(h, w, 0.5 * constants::pi);
    CHECK(std::abs(beam.heave_m) == Approx(1.0).epsilon(0.02));
    CHECK(std::abs(beam.roll_rad) == Approx(k).epsilon(0.02));
    CHECK(std::abs(beam.pitch_rad) < 1e-6 * k);
}

TEST_CASE("Short waves average out over the hull", "[deck_motion]") {
    const auto h = test_hull();
    const double k = constants::two_pi / h.length_m;  // k L = 2 pi
    const double w = std::sqrt(g * k);
    const auto shortw = contact::deck_response(h, w, 0.0);
    const auto longw = contact::deck_response(h, 0.05 * heave_omega(h), 0.0);
    CHECK(std::abs(shortw.heave_m) < 1e-3 * std::abs(longw.heave_m));
    CHECK(std::abs(contact::deck_plane_fit(h, k, 0.0).heave_m) < 1e-12);
}

TEST_CASE("The analytic plane fit matches quadrature", "[deck_motion]") {
    // Midpoint sums at 200 and 400 cells per side, Richardson-extrapolated (the midpoint error is O(h^2); 200 cells
    // alone are only good to ~(k h)^2 / 24 ~ 1e-4 for these waves).
    const auto h = test_hull();
    const double pairs[5][2] = {{0.4, 0.3}, {0.6, 1.0}, {0.8, 2.5}, {1.0, -0.7}, {1.3, 1.47}};
    for (const auto& p : pairs) {
        const double k = p[0] * p[0] / g, a = p[1];
        const auto fit = contact::deck_plane_fit(h, k, a);
        const auto q1 = midpoint_plane(h, k, a, 200), q2 = midpoint_plane(h, k, a, 400);
        const auto rich = [](std::complex<double> c1, std::complex<double> c2) { return (4.0 * c2 - c1) / 3.0; };
        const auto qh = rich(q1.heave_m, q2.heave_m), qr = rich(q1.roll_rad, q2.roll_rad),
                   qp = rich(q1.pitch_rad, q2.pitch_rad);
        INFO("omega " << p[0] << " angle " << a << ": heave " << fit.heave_m << " vs " << qh << ", roll "
                      << fit.roll_rad << " vs " << qr << ", pitch " << fit.pitch_rad << " vs " << qp);
        CHECK(std::abs(fit.heave_m - qh) <= 1e-6 * std::abs(qh));
        CHECK(std::abs(fit.roll_rad - qr) <= 1e-6 * std::abs(qr));
        CHECK(std::abs(fit.pitch_rad - qp) <= 1e-6 * std::abs(qp));
    }
}

TEST_CASE("Natural periods follow the hydrostatics", "[deck_motion]") {
    const auto h = test_hull();
    const contact::DeckMotion deck(environment::SeaState(environment::SeaStateConfig{}), h, 0.0);
    const auto n = deck.natural_periods();
    std::printf("deck natural periods: heave %.3f s, roll %.3f s, pitch %.3f s\n", n.heave_s, n.roll_s, n.pitch_s);
    const double d = h.draft_m;
    const double heave = constants::two_pi * std::sqrt((1.0 + h.heave_added_mass_factor) * d / g);
    const double roll = constants::two_pi * std::sqrt((1.0 + h.roll_added_inertia_factor) * h.roll_gyration_m *
                                                      h.roll_gyration_m / (g * metacentric_height(h.beam_m, d)));
    const double pitch = constants::two_pi * std::sqrt((1.0 + h.pitch_added_inertia_factor) * h.pitch_gyration_m *
                                                       h.pitch_gyration_m / (g * metacentric_height(h.length_m, d)));
    CHECK(n.heave_s == Approx(heave).epsilon(1e-9));
    CHECK(n.roll_s == Approx(roll).epsilon(1e-9));
    CHECK(n.pitch_s == Approx(pitch).epsilon(1e-9));

    // At resonance the relative-motion excitation (Korvin-Kroukovsky strip theory) gives the quasi-static plane fit
    // times the Smith factor exp(-k d) times |1 - ca / (1 + ca) - 2 i zeta| / (2 zeta).
    const auto amplification = [&](double period_s, double angle, auto part) {
        const double w = constants::two_pi / period_s;
        return std::abs(part(contact::deck_response(h, w, angle))) /
               std::abs(part(contact::deck_plane_fit(h, w * w / g, angle)));
    };
    const auto expected = [&](double period_s, double ca, double zeta) {
        const double k = std::pow(constants::two_pi / period_s, 2) / g;
        return std::exp(-k * d) * std::abs(std::complex<double>(1.0 - ca / (1.0 + ca), -2.0 * zeta)) / (2.0 * zeta);
    };
    const double oblique = 0.6;
    const double a_heave = amplification(n.heave_s, oblique, [](const auto& r) { return r.heave_m; });
    const double a_roll = amplification(n.roll_s, oblique, [](const auto& r) { return r.roll_rad; });
    const double a_pitch = amplification(n.pitch_s, oblique, [](const auto& r) { return r.pitch_rad; });
    std::printf("deck resonance amplification: heave %.3f, roll %.3f, pitch %.3f\n", a_heave, a_roll, a_pitch);
    CHECK(a_heave == Approx(expected(n.heave_s, h.heave_added_mass_factor, h.heave_damping_ratio)).epsilon(1e-6));
    CHECK(a_roll == Approx(expected(n.roll_s, h.roll_added_inertia_factor, h.roll_damping_ratio)).epsilon(1e-6));
    CHECK(a_pitch == Approx(expected(n.pitch_s, h.pitch_added_inertia_factor, h.pitch_damping_ratio)).epsilon(1e-6));
}

TEST_CASE("The deck follows the sea surface in very long waves", "[deck_motion]") {
    // Swell far below every natural frequency: the deck centre rides the elevation there and tilts with its slopes.
    const auto h = test_hull();
    const double deck_heading = 0.4;
    const environment::SeaState sea(environment::SeaStateConfig{2.0, 600.0, 1.1, 7});
    const contact::DeckMotion deck(sea, h, deck_heading);
    const double bow_e = std::sin(deck_heading), bow_n = std::cos(deck_heading);
    const double port_e = -std::cos(deck_heading), port_n = std::sin(deck_heading);
    const double dx = 1.0;
    for (double t : {0.0, 37.0, 123.4, 401.0, 977.0}) {
        const auto p = deck.pose(t);
        INFO("t " << t);
        CHECK(p.heave_m == Approx(sea.elevation_m(0.0, 0.0, t)).margin(3e-3 * 2.0));
        const double bow_slope =
            (sea.elevation_m(bow_e * dx, bow_n * dx, t) - sea.elevation_m(-bow_e * dx, -bow_n * dx, t)) / (2 * dx);
        const double port_slope =
            (sea.elevation_m(port_e * dx, port_n * dx, t) - sea.elevation_m(-port_e * dx, -port_n * dx, t)) / (2 * dx);
        CHECK(p.pitch_rad == Approx(bow_slope).margin(3e-3 * 1e-3));
        CHECK(p.roll_rad == Approx(port_slope).margin(3e-3 * 1e-3));
    }
}

TEST_CASE("Deck statistics match the time series", "[deck_motion]") {
    const auto h = test_hull();
    const environment::SeaState sea(environment::SeaStateConfig{2.5, 8.0, 1.2, 3});
    const contact::DeckMotion deck(sea, h, 0.3);  // waves 52 deg off the bow
    double heave2 = 0.0, rate2 = 0.0, tilt2 = 0.0;
    const double dt = 0.25;
    const int n = static_cast<int>(3600.0 / dt);
    for (int i = 0; i < n; ++i) {
        const auto p = deck.pose(i * dt);
        heave2 += p.heave_m * p.heave_m;
        rate2 += p.heave_rate_mps * p.heave_rate_mps;
        tilt2 += p.roll_rad * p.roll_rad + p.pitch_rad * p.pitch_rad;
    }
    const auto s = deck.statistics();
    std::printf("deck statistics Hs 2.5 m Tp 8 s: heave %.3f m, heave rate %.3f m/s, tilt %.3f deg (2 sigma)\n",
                s.significant_heave_m, s.significant_heave_rate_mps, s.significant_tilt_rad / constants::deg_to_rad);
    // 2 sigma of each series (zero mean); the tilt is the RMS of the combined roll / pitch angle
    CHECK(2.0 * std::sqrt(heave2 / n) == Approx(s.significant_heave_m).epsilon(0.10));
    CHECK(2.0 * std::sqrt(rate2 / n) == Approx(s.significant_heave_rate_mps).epsilon(0.10));
    CHECK(2.0 * std::sqrt(tilt2 / n) == Approx(s.significant_tilt_rad).epsilon(0.10));
    CHECK(s.significant_heave_m > 0.0);
    // for the limits (Task 4): the same sea from ahead, quartering and abeam
    for (double off_deg : {0.0, 45.0, 90.0}) {
        const auto q = contact::DeckMotion(sea, h, 1.2 - off_deg * constants::deg_to_rad).statistics();
        std::printf("  waves %2.0f deg off the bow: heave %.3f m, heave rate %.3f m/s, tilt %.3f deg\n", off_deg,
                    q.significant_heave_m, q.significant_heave_rate_mps, q.significant_tilt_rad / constants::deg_to_rad);
    }
}

TEST_CASE("The deck frame is a rigid motion consistent with the pose", "[deck_motion]") {
    const auto h = test_hull();
    const environment::SeaState sea(environment::SeaStateConfig{2.5, 8.0, 2.0, 11});
    const contact::DeckMotion deck(sea, h, -0.5);
    const double dt = 1e-4;
    for (double t : {3.0, 61.7, 250.25}) {
        INFO("t " << t);
        const auto f = deck.frame(t);
        const auto f0 = deck.frame(t - 0.5 * dt), f1 = deck.frame(t + 0.5 * dt);
        const auto p = deck.pose(t);
        CHECK(f.origin_site_m.x == 0.0);
        CHECK(f.origin_site_m.y == 0.0);
        CHECK(f.origin_site_m.z == Approx(p.heave_m));
        const auto v = (f1.origin_site_m - f0.origin_site_m) / dt;
        CHECK(math::norm(v - f.velocity_site_mps) < 1e-5);
        // orthonormal, right-handed
        const auto& r = f.deck_to_site;
        CHECK(math::determinant(r) == Approx(1.0).margin(1e-12));
        CHECK(math::dot(math::column(r, 0), math::column(r, 1)) == Approx(0.0).margin(1e-12));
        // dR/dt R^T = [w]x
        math::Mat3 rdot;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) rdot.m[i][j] = (f1.deck_to_site.m[i][j] - f0.deck_to_site.m[i][j]) / dt;
        const auto wx = rdot * math::transpose(r);
        const math::Vec3 w{wx.m[2][1], wx.m[0][2], wx.m[1][0]};
        CHECK(math::norm(w - f.angular_velocity_site_radps) < 1e-5);
        // the deck normal tilts by the pose's angles: bow up by pitch, port up by roll
        CHECK(std::asin(math::column(r, 0).z) == Approx(p.pitch_rad).margin(1e-12));
    }
    const contact::DeckMotion calm(environment::SeaState(environment::SeaStateConfig{}), h, 0.7);
    const auto c = calm.frame(10.0);
    CHECK(c.origin_site_m.z == 0.0);
    CHECK(math::norm(c.velocity_site_mps) == 0.0);
    CHECK(math::norm(c.angular_velocity_site_radps) == 0.0);
    // level frame: bow along the deck heading (clockwise from north), port to its left, normal up
    CHECK(math::column(c.deck_to_site, 0).x == Approx(std::sin(0.7)));
    CHECK(math::column(c.deck_to_site, 0).y == Approx(std::cos(0.7)));
    CHECK(math::column(c.deck_to_site, 1).x == Approx(-std::cos(0.7)));
    CHECK(math::column(c.deck_to_site, 2).z == 1.0);
}
