#pragma once

#include <array>
#include <cmath>
#include <cstddef>

namespace starsim::math {

// Fixed-size double vector: the state type of all ODE integrators.
template <std::size_t N>
struct VecN {
    std::array<double, N> v{};

    static constexpr std::size_t size() { return N; }

    constexpr double& operator[](std::size_t i) { return v[i]; }
    constexpr double operator[](std::size_t i) const { return v[i]; }

    constexpr VecN& operator+=(const VecN& o) {
        for (std::size_t i = 0; i < N; ++i) v[i] += o.v[i];
        return *this;
    }
    constexpr VecN& operator-=(const VecN& o) {
        for (std::size_t i = 0; i < N; ++i) v[i] -= o.v[i];
        return *this;
    }
    constexpr VecN& operator*=(double s) {
        for (auto& x : v) x *= s;
        return *this;
    }

    friend constexpr VecN operator+(VecN a, const VecN& b) { return a += b; }
    friend constexpr VecN operator-(VecN a, const VecN& b) { return a -= b; }
    friend constexpr VecN operator*(VecN a, double s) { return a *= s; }
    friend constexpr VecN operator*(double s, VecN a) { return a *= s; }
    friend constexpr bool operator==(const VecN&, const VecN&) = default;

    // a += s * b without temporaries (hot path of the Runge-Kutta stages).
    constexpr VecN& add_scaled(double s, const VecN& b) {
        for (std::size_t i = 0; i < N; ++i) v[i] += s * b.v[i];
        return *this;
    }

    static constexpr VecN filled(double value) {
        VecN r;
        r.v.fill(value);
        return r;
    }
};

}  // namespace starsim::math
