// Finite burn model: the equivalent impulsive state of a constant-thrust burn centred on the node.

#include <cmath>

#include <catch2/catch_test_macros.hpp>

#include "starsim/environment/gravity.hpp"
#include "starsim/mission/finite_burn.hpp"

using namespace starsim;
using math::Vec3;

namespace {
struct NoGravity : environment::GravityModel {
    ephemeris::Body central_body() const override { return ephemeris::Body::Earth; }
    Vec3 acceleration_mps2(const Vec3&, time::Epoch) const override { return {}; }
};
}  // namespace

TEST_CASE("Finite burn without gravity: rocket equation velocity, analytic displacement", "[mission][finite_burn]") {
    const NoGravity gravity;
    const math::StateVector at_node{{7.0e6, 0.0, 0.0}, {0.0, 7500.0, 0.0}};
    const mission::FiniteBurnModel engines{1.72e6, 8.94e6, 2400.0};
    const Vec3 dv{0.0, 3000.0, 400.0};
    const auto t0 = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);
    const auto s = mission::apply_finite_burn(gravity, at_node, t0, dv, engines);
    // The full delta-v is gained (rocket equation duration).
    CHECK(norm(s.velocity_mps - (at_node.velocity_mps + dv)) < 1e-6);
    // Powered displacement int_0^T ve ln(m0 / m(t)) dt = ve (T - m1 / mdot ln(m0 / m1)), centred on the node: the
    // equivalent state is offset by that minus dv T / 2.
    const double ve = engines.thrust_n / engines.mass_flow_kgps;
    const double m1 = engines.mass_kg * std::exp(-norm(dv) / ve);
    const double T = (engines.mass_kg - m1) / engines.mass_flow_kgps;
    const double powered = ve * (T - m1 / engines.mass_flow_kgps * std::log(engines.mass_kg / m1));
    const Vec3 u = dv / norm(dv);
    const Vec3 expected = at_node.position_m + u * (powered - 0.5 * norm(dv) * T);
    CHECK(norm(s.position_m - expected) < 1e-3);
}

TEST_CASE("Finite burn: a short burn is the impulse", "[mission][finite_burn]") {
    const NoGravity gravity;
    const math::StateVector at_node{{7.0e6, 0.0, 0.0}, {0.0, 7500.0, 0.0}};
    const auto s = mission::apply_finite_burn(gravity, at_node, time::Epoch::from_tdb_seconds_past_j2000(8.4e8),
                                              {0.0, 1.0, 0.0}, {1.72e6, 8.94e6, 2400.0});
    CHECK(norm(s.velocity_mps - Vec3{0.0, 7501.0, 0.0}) < 1e-9);
    CHECK(norm(s.position_m - at_node.position_m) < 1e-3);  // 0.19 s burn: ~ dv T^2 / 24
}
