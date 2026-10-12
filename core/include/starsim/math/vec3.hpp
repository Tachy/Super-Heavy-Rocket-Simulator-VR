#pragma once

#include <cmath>

namespace starsim::math {

// Double-precision 3-vector. Plain aggregate; frame and unit are carried by variable names.
struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    constexpr Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    constexpr Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    constexpr Vec3& operator*=(double s) { x *= s; y *= s; z *= s; return *this; }
    constexpr Vec3& operator/=(double s) { x /= s; y /= s; z /= s; return *this; }

    friend constexpr Vec3 operator+(Vec3 a, const Vec3& b) { return a += b; }
    friend constexpr Vec3 operator-(Vec3 a, const Vec3& b) { return a -= b; }
    friend constexpr Vec3 operator-(const Vec3& a) { return {-a.x, -a.y, -a.z}; }
    friend constexpr Vec3 operator*(Vec3 a, double s) { return a *= s; }
    friend constexpr Vec3 operator*(double s, Vec3 a) { return a *= s; }
    friend constexpr Vec3 operator/(Vec3 a, double s) { return a /= s; }
    friend constexpr bool operator==(const Vec3&, const Vec3&) = default;
};

constexpr double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

constexpr Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

inline double norm(const Vec3& v) { return std::sqrt(dot(v, v)); }

inline Vec3 normalized(const Vec3& v) { return v / norm(v); }

}  // namespace starsim::math
