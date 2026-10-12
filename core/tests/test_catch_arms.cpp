#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "starsim/config/launch_site_config.hpp"
#include "starsim/contact/catch_arms.hpp"

using namespace starsim;
using Catch::Matchers::WithinAbs;

namespace {

config::TowerConfig tower() { return *config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_olp1").tower; }

void run(contact::CatchArms& arms, double seconds, double push_n, double load_n) {
    for (int i = 0; i < static_cast<int>(seconds / 0.005 + 0.5); ++i) arms.step(0.005, push_n, load_n);
}

}  // namespace

TEST_CASE("Arms close at their rate limit", "[contact]") {
    const auto config = tower();
    contact::CatchArms arms(config);
    CHECK_THAT(arms.opening_rad(), WithinAbs(config.arm_open_rad, 1e-15));
    CHECK_THAT(arms.closing_time_s(), WithinAbs(4.0, 1e-12));
    arms.command_close(true);
    run(arms, 2.0, 0.0, 0.0);
    CHECK_THAT(arms.opening_rad(), WithinAbs(0.5 * config.arm_open_rad, 1e-9));
    CHECK_THAT(arms.opening_rate_rad_s(), WithinAbs(-config.arm_rate_rad_s, 1e-9));
    run(arms, 2.1, 0.0, 0.0);
    CHECK(arms.closed());
    CHECK(arms.opening_rad() == 0.0);
}

TEST_CASE("Arms stall when the hull pushes back", "[contact]") {
    const auto config = tower();
    contact::CatchArms arms(config);
    arms.command_close(true);
    run(arms, 1.0, 0.0, 0.0);
    const double before = arms.opening_rad();
    run(arms, 1.0, 2.0 * config.arm_force_limit_n, 0.0);
    CHECK(arms.stalled());
    CHECK(arms.opening_rad() == before);
}

TEST_CASE("Carriage shock absorbers carry a static load and stop at the end of their stroke", "[contact]") {
    const auto config = tower();
    contact::CatchArms arms(config);
    run(arms, 30.0, 0.0, 1.0e6);
    CHECK_THAT(arms.carriage_offset_m(), WithinAbs(-1.0e6 / config.carriage_stiffness_n_m, 1e-3));
    CHECK_FALSE(arms.hit_hard_stop());
    contact::CatchArms overloaded(config);
    run(overloaded, 5.0, 0.0, 5.0e7);
    CHECK(overloaded.hit_hard_stop());
    CHECK(overloaded.carriage_offset_m() == -config.carriage_stroke_m);
}

TEST_CASE("Arms can be set closed instantly", "[contact]") {
    contact::CatchArms arms(tower());
    arms.set_closed();
    CHECK(arms.closed());
    CHECK(arms.close_commanded());
}
