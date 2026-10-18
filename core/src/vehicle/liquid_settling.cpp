#include "starsim/vehicle/liquid_settling.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <mutex>

#include "starsim/math/constants.hpp"

namespace starsim::vehicle {
namespace {

using constants::pi;

constexpr size_t tilt_rows = 181;          // 1 deg steps over 0..180 deg
constexpr size_t table_offsets = 121;      // plane offsets per row
constexpr size_t table_quadrature = 100;   // strips / slices per integral
constexpr size_t direct_quadrature = 400;
constexpr double tilt_step_rad = pi / static_cast<double>(tilt_rows - 1);

struct RawMoments {
    double v = 0.0, mx = 0.0, mu = 0.0, sxx = 0.0, suu = 0.0, sww = 0.0;
};

// Range of the height h = x cos t - u sin t (up = -down) over the cylinder.
double height_extent(double aspect, double tilt) {
    return 0.5 * aspect * std::abs(std::cos(tilt)) + std::sin(tilt);
}

// Liquid below the plane h = c. Near the axis (|cos t| >= sin t) the integral runs over strips across the axis
// (u = -cos phi: smooth at the rim), each strip's liquid an axial interval - exact for the aft column. Otherwise over
// axial slices, each slice's liquid an analytic circular segment u >= u0.
RawMoments integrate(double aspect, double tilt, double c, size_t n) {
    RawMoments m;
    const double ct = std::cos(tilt), st = std::sin(tilt);
    const double half = 0.5 * aspect;
    if (std::abs(ct) >= st) {
        const double dphi = pi / static_cast<double>(n);
        for (size_t k = 0; k < n; ++k) {
            const double phi = (static_cast<double>(k) + 0.5) * dphi;
            const double u = -std::cos(phi), s = std::sin(phi);
            const double x1 = (c + u * st) / ct;
            const double xa = ct > 0.0 ? -half : std::max(-half, x1);
            const double xb = ct > 0.0 ? std::min(half, x1) : half;
            if (xb <= xa) continue;
            const double len = xb - xa;
            const double area = 2.0 * s * s * dphi;  // chord 2 s times du = s dphi
            m.v += area * len;
            m.mx += area * 0.5 * (xb * xb - xa * xa);
            m.sxx += area * (xb * xb * xb - xa * xa * xa) / 3.0;
            m.mu += area * u * len;
            m.suu += area * u * u * len;
            m.sww += (2.0 / 3.0) * s * s * s * s * dphi * len;
        }
        return m;
    }
    const double dx = aspect / static_cast<double>(n);
    for (size_t k = 0; k < n; ++k) {
        const double x = -half + (static_cast<double>(k) + 0.5) * dx;
        const double u0 = std::clamp((x * ct - c) / st, -1.0, 1.0);
        if (u0 >= 1.0) continue;
        const double s0 = std::sqrt(1.0 - u0 * u0);
        const double asin_u0 = std::asin(u0);
        const double area = std::acos(u0) - u0 * s0;
        const double first_u = (2.0 / 3.0) * s0 * s0 * s0;
        const double second_u = 0.25 * (0.5 * pi - asin_u0 + u0 * s0 * (1.0 - 2.0 * u0 * u0));
        // int_{u0}^{1} (2/3)(1 - u^2)^(3/2) du with H(u) = u (5 - 2u^2) sqrt(1 - u^2) / 8 + 3/8 asin u, H(1) = 3 pi / 16
        const double h_u0 = u0 * (5.0 - 2.0 * u0 * u0) * s0 / 8.0 + 0.375 * asin_u0;
        const double second_w = (2.0 / 3.0) * (3.0 * pi / 16.0 - h_u0);
        m.v += area * dx;
        m.mx += x * area * dx;
        m.sxx += (x * x + dx * dx / 12.0) * area * dx;
        m.mu += first_u * dx;
        m.suu += second_u * dx;
        m.sww += second_w * dx;
    }
    return m;
}

LiquidMoments moments_of(const RawMoments& r) {
    if (r.v <= 0.0) return {};
    LiquidMoments out;
    out.axial = r.mx / r.v;
    out.lateral = r.mu / r.v;
    out.axial_var = std::max(0.0, r.sxx / r.v - out.axial * out.axial);
    out.lateral_var = std::max(0.0, r.suu / r.v - out.lateral * out.lateral);
    out.transverse_var = std::max(0.0, r.sww / r.v);
    return out;
}

}  // namespace

SettlingTable::SettlingTable(double aspect) : aspect_(aspect), offsets_(table_offsets) {
    raw_.resize(tilt_rows * offsets_);
    for (size_t row = 0; row < tilt_rows; ++row) {
        const double tilt = static_cast<double>(row) * tilt_step_rad;
        const double extent = height_extent(aspect, tilt);
        for (size_t j = 0; j < offsets_; ++j) {
            const double c = -extent + 2.0 * extent * static_cast<double>(j) / static_cast<double>(offsets_ - 1);
            const auto m = integrate(aspect, tilt, c, table_quadrature);
            raw_[row * offsets_ + j] = {m.v, m.mx, m.mu, m.sxx, m.suu, m.sww};
        }
    }
}

SettlingTable::Raw SettlingTable::row_at(size_t row, double volume) const {
    if (row == 0 || row == tilt_rows - 1) {
        // Down along the axis: a column against an end, exact (the moments are not linear in the plane offset).
        const double h = std::min(volume / pi, aspect_);
        const double xa = row == 0 ? -0.5 * aspect_ : 0.5 * aspect_ - h;
        const double xb = xa + h;
        return {pi * h, 0.5 * pi * (xb * xb - xa * xa), 0.0, pi * (xb * xb * xb - xa * xa * xa) / 3.0, 0.25 * pi * h,
                0.25 * pi * h};
    }
    const Raw* r = &raw_[row * offsets_];
    if (volume <= r[0].v) return r[0];
    if (volume >= r[offsets_ - 1].v) return r[offsets_ - 1];
    size_t lo = 0, hi = offsets_ - 1;
    while (hi - lo > 1) {
        const size_t mid = (lo + hi) / 2;
        (r[mid].v < volume ? lo : hi) = mid;
    }
    const double span = r[hi].v - r[lo].v;
    const double w = span > 0.0 ? (volume - r[lo].v) / span : 0.0;
    const auto lerp = [w](double a, double b) { return a + w * (b - a); };
    return {volume, lerp(r[lo].mx, r[hi].mx), lerp(r[lo].mu, r[hi].mu), lerp(r[lo].sxx, r[hi].sxx),
            lerp(r[lo].suu, r[hi].suu), lerp(r[lo].sww, r[hi].sww)};
}

LiquidMoments SettlingTable::query(double fill, double tilt_rad) const {
    const double volume = std::clamp(fill, 0.0, 1.0) * pi * aspect_;
    if (volume <= 0.0) return {};
    const double pos = std::clamp(tilt_rad, 0.0, pi) / tilt_step_rad;
    const size_t row = std::min(static_cast<size_t>(pos), tilt_rows - 2);
    const double w = pos - static_cast<double>(row);
    const Raw a = row_at(row, volume), b = row_at(row + 1, volume);
    const auto lerp = [w](double x, double y) { return x + w * (y - x); };
    return moments_of({volume, lerp(a.mx, b.mx), lerp(a.mu, b.mu), lerp(a.sxx, b.sxx), lerp(a.suu, b.suu),
                       lerp(a.sww, b.sww)});
}

std::shared_ptr<const SettlingTable> settling_table(double aspect) {
    static std::mutex mutex;
    static std::map<double, std::shared_ptr<const SettlingTable>> cache;
    const std::lock_guard lock(mutex);
    auto& entry = cache[aspect];
    if (!entry) entry = std::make_shared<const SettlingTable>(aspect);
    return entry;
}

LiquidMoments settle_direct(double aspect, double fill, double tilt_rad) {
    const double tilt = std::clamp(tilt_rad, 0.0, pi);
    const double target = std::clamp(fill, 0.0, 1.0) * pi * aspect;
    if (target <= 0.0) return {};
    double lo = -height_extent(aspect, tilt), hi = -lo;
    for (int i = 0; i < 100; ++i) {
        const double mid = 0.5 * (lo + hi);
        (integrate(aspect, tilt, mid, direct_quadrature).v < target ? lo : hi) = mid;
    }
    return moments_of(integrate(aspect, tilt, 0.5 * (lo + hi), direct_quadrature));
}

}  // namespace starsim::vehicle
