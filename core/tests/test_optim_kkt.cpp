#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <span>
#include <vector>

#include "starsim/optim/kkt_system.hpp"

using namespace starsim::optim;

TEST_CASE("KKT solve satisfies the unregularised system", "[optim]") {
    const CscMatrix a(1, 3, {{0, 0, 1.0}, {0, 1, 1.0}, {0, 2, 1.0}});
    const CscMatrix g(4, 3, {{0, 0, -1.0}, {1, 2, -1.0}, {1, 1, 0.5}, {2, 0, -1.0}, {3, 1, -1.0}});
    ConeLayout cones;
    cones.nonnegative = 1;
    cones.second_order = {3};
    KktSystem kkt(a, g, cones, 7e-8, 8);
    REQUIRE(kkt.size() == 8);
    NtScaling scaling(cones);
    REQUIRE(scaling.update(std::vector<double>{0.5, 2.0, 0.3, -0.4}, std::vector<double>{1.5, 1.0, 0.2, 0.1}));
    REQUIRE(kkt.factor(scaling));

    std::vector<double> rhs(8), u(8);
    for (int i = 0; i < 8; ++i) rhs[i] = std::cos(i + 1.0);
    kkt.solve(rhs, u);

    const std::span<const double> ux(u.data(), 3), uy(u.data() + 3, 1), uz(u.data() + 4, 4);
    std::vector<double> r(8, 0.0);
    // Row block x: A^T uy + G^T uz
    a.multiply_transpose_add(uy, std::span<double>(r.data(), 3));
    g.multiply_transpose_add(uz, std::span<double>(r.data(), 3));
    // Row block y: A ux
    a.multiply_add(ux, std::span<double>(r.data() + 3, 1));
    // Row block z: G ux - W^T W uz
    g.multiply_add(ux, std::span<double>(r.data() + 4, 4));
    std::vector<double> wz(4), wwz(4);
    scaling.apply(uz, wz);
    scaling.apply(wz, wwz);
    for (int i = 0; i < 4; ++i) r[4 + i] -= wwz[i];
    for (int i = 0; i < 8; ++i) r[i] -= rhs[i];
    CHECK(norm_inf(r) <= 1e-10);
}
