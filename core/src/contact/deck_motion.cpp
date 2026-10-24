#include "starsim/contact/deck_motion.hpp"

#include <cmath>

#include "starsim/math/constants.hpp"

namespace starsim::contact {

namespace {

using cplx = std::complex<double>;
constexpr double g = constants::standard_gravity_mps2;
constexpr double series_below = 1e-3;  // |u| below which the sinc factors use their Taylor series

// Mean of exp(i 2u s / w) over s in [-w/2, w/2]: sin(u) / u.
double sinc(double u) {
    if (std::abs(u) < series_below) return 1.0 - u * u / 6.0 + u * u * u * u / 120.0;
    return std::sin(u) / u;
}

// d sinc / du = (u cos u - sin u) / u^2.
double sinc_derivative(double u) {
    if (std::abs(u) < series_below) return -u / 3.0 + u * u * u / 30.0;
    return (u * std::cos(u) - std::sin(u)) / (u * u);
}

// Least-squares slope of exp(i kappa s) along a side of width w, s in [-w/2, w/2]: <s e> / <s^2> with <s^2> = w^2 / 12
// and <s e> = -i d/dkappa sinc(kappa w / 2) = -i (w / 2) sinc'(u).
cplx slope_factor(double u, double width_m) { return cplx(0.0, -6.0 / width_m) * sinc_derivative(u); }

// One mode under the strip-theory relative-motion excitation (Korvin-Kroukovsky): the hydrostatic, added-mass and
// damping forces act on the motion relative to the (Smith-reduced) wave, so with time factor exp(-i omega t)
//   (c - (m + a) omega^2 - i omega b) x = (c - a omega^2 - i omega b) exp(-k d) eta_fit
// and per unit plane fit: exp(-k d) (1 - r^2 ca / (1 + ca) - 2 i zeta r) / (1 - r^2 - 2 i zeta r); 1 at omega -> 0.
cplx mode_transfer(double omega, double omega_n, double added_factor, double zeta, double smith) {
    const double r = omega / omega_n;
    const cplx excitation(1.0 - r * r * added_factor / (1.0 + added_factor), -2.0 * zeta * r);
    return smith * excitation / cplx(1.0 - r * r, -2.0 * zeta * r);
}

// Box barge, KG at the waterline: GM = KB + BM - KG = d/2 + w^2 / (12 d) - d.
double metacentric_height_m(double width_m, double draft_m) {
    return width_m * width_m / (12.0 * draft_m) - 0.5 * draft_m;
}

double heave_omega(const config::DroneShipHull& h) {
    return std::sqrt(g / ((1.0 + h.heave_added_mass_factor) * h.draft_m));
}

double roll_omega(const config::DroneShipHull& h) {
    return std::sqrt(g * metacentric_height_m(h.beam_m, h.draft_m) /
                     ((1.0 + h.roll_added_inertia_factor) * h.roll_gyration_m * h.roll_gyration_m));
}

double pitch_omega(const config::DroneShipHull& h) {
    return std::sqrt(g * metacentric_height_m(h.length_m, h.draft_m) /
                     ((1.0 + h.pitch_added_inertia_factor) * h.pitch_gyration_m * h.pitch_gyration_m));
}

double real_at(cplx amplitude, double omega, double t_s) { return (amplitude * std::polar(1.0, -omega * t_s)).real(); }

}  // namespace

DeckResponse deck_plane_fit(const config::DroneShipHull& hull, double wavenumber_radpm, double wave_angle_rad) {
    // Wave in deck axes (x bow, y port): exp(i k (x cos a - y sin a)); separable over the rectangle, and 1, x, y are
    // orthogonal on it, so the plane's three coefficients are independent products of the side factors.
    const double ux = 0.5 * wavenumber_radpm * std::cos(wave_angle_rad) * hull.length_m;
    const double uy = -0.5 * wavenumber_radpm * std::sin(wave_angle_rad) * hull.beam_m;
    const double fx = sinc(ux), fy = sinc(uy);
    return {cplx(fx * fy), fx * slope_factor(uy, hull.beam_m), slope_factor(ux, hull.length_m) * fy};
}

DeckResponse deck_response(const config::DroneShipHull& hull, double omega_radps, double wave_angle_rad) {
    const double k = omega_radps * omega_radps / g;
    const auto fit = deck_plane_fit(hull, k, wave_angle_rad);
    // Smith effect: the wave pressure at the keel decays as exp(-k d) (the usual strip-theory reduction for the heave of
    // a flat-bottomed section, applied to all its excitation terms); for roll and pitch a crude approximation (the moment also has side-wall pressure terms nearer the surface) [call].
    const double smith = std::exp(-k * hull.draft_m);
    return {fit.heave_m * mode_transfer(omega_radps, heave_omega(hull), hull.heave_added_mass_factor,
                                        hull.heave_damping_ratio, smith),
            fit.roll_rad * mode_transfer(omega_radps, roll_omega(hull), hull.roll_added_inertia_factor,
                                         hull.roll_damping_ratio, smith),
            fit.pitch_rad * mode_transfer(omega_radps, pitch_omega(hull), hull.pitch_added_inertia_factor,
                                          hull.pitch_damping_ratio, smith)};
}

DeckMotion::DeckMotion(const environment::SeaState& sea, const config::DroneShipHull& hull, double deck_heading_rad)
    : hull_(hull), deck_heading_rad_(deck_heading_rad) {
    const double wave_angle = sea.config().heading_rad - deck_heading_rad;
    for (const auto& c : sea.components()) {
        // SeaState's components use k = omega^2 / g, as deck_response
        const auto r = deck_response(hull, c.angular_frequency_radps, wave_angle);
        const cplx a = std::polar(c.amplitude_m, c.phase_rad);
        modes_.push_back({c.angular_frequency_radps, a * r.heave_m, a * r.roll_rad, a * r.pitch_rad});
    }
}

DeckPose DeckMotion::pose(double t_s) const {
    DeckPose p;
    for (const auto& m : modes_) {
        const double w = m.omega_radps;
        const cplx d_dt(0.0, -w);
        p.heave_m += real_at(m.heave_m, w, t_s);
        p.roll_rad += real_at(m.roll_rad, w, t_s);
        p.pitch_rad += real_at(m.pitch_rad, w, t_s);
        p.heave_rate_mps += real_at(d_dt * m.heave_m, w, t_s);
        p.roll_rate_radps += real_at(d_dt * m.roll_rad, w, t_s);
        p.pitch_rate_radps += real_at(d_dt * m.pitch_rad, w, t_s);
    }
    return p;
}

DeckFrame DeckMotion::frame(double t_s) const {
    const DeckPose p = pose(t_s);
    // level deck axes in the site frame: bow along the heading (clockwise from north), port to its left, up
    const math::Vec3 bow{std::sin(deck_heading_rad_), std::cos(deck_heading_rad_), 0.0};
    const math::Vec3 port{-std::cos(deck_heading_rad_), std::sin(deck_heading_rad_), 0.0};
    const math::Mat3 level = math::Mat3::from_columns(bow, port, {0.0, 0.0, 1.0});
    // roll phi about the bow axis (port up), then pitch about the level port axis: a right-handed rotation by
    // alpha = -pitch raises the bow
    const double cr = std::cos(p.roll_rad), sr = std::sin(p.roll_rad);
    const double alpha = -p.pitch_rad, ca = std::cos(alpha), sa = std::sin(alpha);
    math::Mat3 rx = math::Mat3::identity();
    rx.m[1] = {0.0, cr, -sr};
    rx.m[2] = {0.0, sr, cr};
    math::Mat3 ry = math::Mat3::identity();
    ry.m[0] = {ca, 0.0, sa};
    ry.m[2] = {-sa, 0.0, ca};

    DeckFrame f;
    f.deck_to_site = level * ry * rx;
    f.origin_site_m = {0.0, 0.0, p.heave_m};
    f.velocity_site_mps = {0.0, 0.0, p.heave_rate_mps};
    // d/dt (Ry Rx): alpha' about y plus phi' about the pitched x axis Ry e_x
    const math::Vec3 w_level = math::Vec3{0.0, -p.pitch_rate_radps, 0.0} + p.roll_rate_radps * math::Vec3{ca, 0.0, -sa};
    f.angular_velocity_site_radps = level * w_level;
    return f;
}

DeckMotionStatistics DeckMotion::statistics() const {
    double heave = 0.0, rate = 0.0, tilt = 0.0;
    for (const auto& m : modes_) {
        heave += 0.5 * std::norm(m.heave_m);
        rate += 0.5 * m.omega_radps * m.omega_radps * std::norm(m.heave_m);
        tilt += 0.5 * (std::norm(m.roll_rad) + std::norm(m.pitch_rad));
    }
    return {2.0 * std::sqrt(heave), 2.0 * std::sqrt(rate), 2.0 * std::sqrt(tilt)};
}

DeckNaturalPeriods DeckMotion::natural_periods() const {
    return {constants::two_pi / heave_omega(hull_), constants::two_pi / roll_omega(hull_),
            constants::two_pi / pitch_omega(hull_)};
}

}  // namespace starsim::contact
