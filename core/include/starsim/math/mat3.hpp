#pragma once

#include <array>

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

}  // namespace starsim::math
