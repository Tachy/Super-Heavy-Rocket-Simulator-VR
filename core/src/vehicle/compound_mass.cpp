#include "starsim/vehicle/compound_mass.hpp"

namespace starsim::vehicle {

namespace {
// Parallel-axis term of a point mass m at offset d: m (|d|^2 E - d d^T).
math::Mat3 parallel_axis(double m, const math::Vec3& d) {
    const double dd = dot(d, d);
    const double v[3] = {d.x, d.y, d.z};
    math::Mat3 r = math::Mat3::identity();
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) r(i, j) = m * ((i == j ? dd : 0.0) - v[i] * v[j]);
    }
    return r;
}
math::Mat3 sum(const math::Mat3& a, const math::Mat3& b) {
    math::Mat3 r = a;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) r(i, j) += b(i, j);
    }
    return r;
}
}  // namespace

CompoundMass combine_mass(double m1, const math::Vec3& cg1, const math::Mat3& i1, double m2, const math::Vec3& cg2,
                          const math::Mat3& i2) {
    CompoundMass c;
    c.mass_kg = m1 + m2;
    c.cg_m = (m1 * cg1 + m2 * cg2) / c.mass_kg;
    c.inertia_kg_m2 = sum(sum(i1, i2), sum(parallel_axis(m1, cg1 - c.cg_m), parallel_axis(m2, cg2 - c.cg_m)));
    return c;
}

}  // namespace starsim::vehicle
