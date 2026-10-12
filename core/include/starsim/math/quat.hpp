#pragma once

#include <cmath>

#include "starsim/math/mat3.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::math {

// Unit quaternion (Hamilton convention, scalar first). Used as attitude: rotates body vectors into
// the reference frame, v_ref = q * v_body * q^-1.
struct Quat {
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    static Quat from_axis_angle(const Vec3& axis, double angle_rad) {
        const Vec3 n = math::normalized(axis);
        const double s = std::sin(0.5 * angle_rad);
        return {std::cos(0.5 * angle_rad), n.x * s, n.y * s, n.z * s};
    }

    // Rotation matrix from orthonormal columns (images of the body x, y, z axes).
    static Quat from_matrix(const Mat3& m) {
        const double trace = m(0, 0) + m(1, 1) + m(2, 2);
        Quat q;
        if (trace > 0.0) {
            const double s = 0.5 / std::sqrt(trace + 1.0);
            q = {0.25 / s, (m(2, 1) - m(1, 2)) * s, (m(0, 2) - m(2, 0)) * s, (m(1, 0) - m(0, 1)) * s};
        } else if (m(0, 0) > m(1, 1) && m(0, 0) > m(2, 2)) {
            const double s = 2.0 * std::sqrt(1.0 + m(0, 0) - m(1, 1) - m(2, 2));
            q = {(m(2, 1) - m(1, 2)) / s, 0.25 * s, (m(0, 1) + m(1, 0)) / s, (m(0, 2) + m(2, 0)) / s};
        } else if (m(1, 1) > m(2, 2)) {
            const double s = 2.0 * std::sqrt(1.0 + m(1, 1) - m(0, 0) - m(2, 2));
            q = {(m(0, 2) - m(2, 0)) / s, (m(0, 1) + m(1, 0)) / s, 0.25 * s, (m(1, 2) + m(2, 1)) / s};
        } else {
            const double s = 2.0 * std::sqrt(1.0 + m(2, 2) - m(0, 0) - m(1, 1));
            q = {(m(1, 0) - m(0, 1)) / s, (m(0, 2) + m(2, 0)) / s, (m(1, 2) + m(2, 1)) / s, 0.25 * s};
        }
        return q.normalized();
    }

    friend Quat operator*(const Quat& a, const Quat& b) {
        return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
    }

    Quat conjugate() const { return {w, -x, -y, -z}; }

    Quat normalized() const {
        const double n = std::sqrt(w * w + x * x + y * y + z * z);
        return {w / n, x / n, y / n, z / n};
    }

    Vec3 rotate(const Vec3& v) const {
        const Vec3 u{x, y, z};
        const Vec3 t = 2.0 * cross(u, v);
        return v + w * t + cross(u, t);
    }

    Vec3 inverse_rotate(const Vec3& v) const { return conjugate().rotate(v); }

    Mat3 to_matrix() const {
        Mat3 m;
        m(0, 0) = 1 - 2 * (y * y + z * z); m(0, 1) = 2 * (x * y - w * z);     m(0, 2) = 2 * (x * z + w * y);
        m(1, 0) = 2 * (x * y + w * z);     m(1, 1) = 1 - 2 * (x * x + z * z); m(1, 2) = 2 * (y * z - w * x);
        m(2, 0) = 2 * (x * z - w * y);     m(2, 1) = 2 * (y * z + w * x);     m(2, 2) = 1 - 2 * (x * x + y * y);
        return m;
    }
};

// Time derivative of an attitude quaternion for a body angular velocity (body axes).
inline Quat attitude_rate(const Quat& q, const Vec3& omega_body) {
    const Quat r = q * Quat{0.0, omega_body.x, omega_body.y, omega_body.z};
    return {0.5 * r.w, 0.5 * r.x, 0.5 * r.y, 0.5 * r.z};
}

}  // namespace starsim::math
