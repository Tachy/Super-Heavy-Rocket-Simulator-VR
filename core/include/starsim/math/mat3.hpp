#pragma once

#include <array>
#include <cmath>
#include <optional>

#include "starsim/math/vec3.hpp"

namespace starsim::math {

// Row-major 3x3 double matrix, used for frame rotations.
struct Mat3 {
    std::array<std::array<double, 3>, 3> m{};

    static constexpr Mat3 identity() {
        Mat3 r;
        r.m[0][0] = r.m[1][1] = r.m[2][2] = 1.0;
        return r;
    }

    static constexpr Mat3 from_columns(const Vec3& c0, const Vec3& c1, const Vec3& c2) {
        Mat3 r;
        r.m[0] = {c0.x, c1.x, c2.x};
        r.m[1] = {c0.y, c1.y, c2.y};
        r.m[2] = {c0.z, c1.z, c2.z};
        return r;
    }

    constexpr double operator()(int row, int col) const { return m[row][col]; }
    constexpr double& operator()(int row, int col) { return m[row][col]; }

    friend constexpr Vec3 operator*(const Mat3& a, const Vec3& v) {
        return {a.m[0][0] * v.x + a.m[0][1] * v.y + a.m[0][2] * v.z,
                a.m[1][0] * v.x + a.m[1][1] * v.y + a.m[1][2] * v.z,
                a.m[2][0] * v.x + a.m[2][1] * v.y + a.m[2][2] * v.z};
    }

    friend constexpr Mat3 operator*(const Mat3& a, const Mat3& b) {
        Mat3 r;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                r.m[i][j] = a.m[i][0] * b.m[0][j] + a.m[i][1] * b.m[1][j] + a.m[i][2] * b.m[2][j];
            }
        }
        return r;
    }
};

constexpr Mat3 transpose(const Mat3& a) {
    Mat3 r;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            r.m[i][j] = a.m[j][i];
        }
    }
    return r;
}

constexpr Vec3 column(const Mat3& a, int k) { return {a.m[0][k], a.m[1][k], a.m[2][k]}; }

constexpr double determinant(const Mat3& a) { return dot(column(a, 0), cross(column(a, 1), column(a, 2))); }

// Solves a x = y by Cramer's rule; nullopt when the columns are (nearly) dependent.
inline std::optional<Vec3> solve(const Mat3& a, const Vec3& y) {
    const Vec3 c0 = column(a, 0), c1 = column(a, 1), c2 = column(a, 2);
    const double det = dot(c0, cross(c1, c2));
    if (!std::isfinite(det) || std::abs(det) <= 1e-14 * norm(c0) * norm(c1) * norm(c2)) return std::nullopt;
    return Vec3{dot(y, cross(c1, c2)), dot(c0, cross(y, c2)), dot(c0, cross(c1, y))} / det;
}

}  // namespace starsim::math
