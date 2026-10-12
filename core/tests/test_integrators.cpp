#include <cmath>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/integrators/integrate.hpp"
#include "starsim/integrators/rk4.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim::integrators;

namespace {

// Harmonic oscillator x'' = -x, x(0) = 1, v(0) = 0  ->  x = cos t, v = -sin t
const OdeFunction<2> oscillator = [](double, const State<2>& y) { return State<2>{{y[1], -y[0]}}; };
const State<2> oscillator_start{{1.0, 0.0}};

Dop853Options<2> tight_options() {
    Dop853Options<2> options;
    options.relative_tolerance = 1e-13;
    options.absolute_tolerance = State<2>::filled(1e-13);
    return options;
}

}  // namespace

TEST_CASE("DOP853 tableau is internally consistent", "[integrators]") {
    double b_sum = 0.0;
    for (double b : dop853::b) b_sum += b;
    CHECK(b_sum == Catch::Approx(1.0).margin(1e-14));
    // Row sums of A equal the nodes C (stage consistency), including the dense-output stages.
    for (int s = 1; s < dop853::n_stages_extended; ++s) {
        if (s == dop853::n_stages) continue;  // row 12 holds B (FSAL stage at c = 1)
        double row = 0.0;
        for (int j = 0; j < s; ++j) row += dop853::a[s][j];
        CHECK(row == Catch::Approx(dop853::c[s]).margin(1e-13));
    }
}

TEST_CASE("DOP853 integrates 10 oscillator periods to 1e-10", "[integrators]") {
    const double t_end = 20.0 * starsim::constants::pi;
    const auto result = integrate<2>(oscillator, 0.0, oscillator_start, t_end, tight_options());
    CHECK(result.t_s == t_end);
    CHECK(std::abs(result.y[0] - std::cos(t_end)) < 1e-10);
    CHECK(std::abs(result.y[1] + std::sin(t_end)) < 1e-10);
}

TEST_CASE("DOP853 dense output is accurate inside steps", "[integrators]") {
    double worst = 0.0;
    const StepObserver<2> observer = [&](const Dop853<2>& solver) {
        for (double x : {0.13, 0.5, 0.91}) {
            const double t = solver.t_previous() + x * (solver.t() - solver.t_previous());
            worst = std::max(worst, std::abs(solver.dense_output(t)[0] - std::cos(t)));
        }
    };
    integrate<2>(oscillator, 0.0, oscillator_start, 10.0, tight_options(), {}, observer);
    CHECK(worst < 1e-10);
}

TEST_CASE("DOP853 integrates backwards to the initial state", "[integrators]") {
    const auto forward = integrate<2>(oscillator, 0.0, oscillator_start, 7.3, tight_options());
    const auto back = integrate<2>(oscillator, 7.3, forward.y, 0.0, tight_options());
    CHECK(back.t_s == 0.0);
    CHECK(std::abs(back.y[0] - 1.0) < 1e-11);
    CHECK(std::abs(back.y[1]) < 1e-11);
}

TEST_CASE("Terminal events are located on the dense output", "[integrators]") {
    // x = cos t falls through zero at pi/2 and rises through zero at 3 pi/2.
    const std::vector<Event<2>> events{
        {"x rising", [](double, const State<2>& y) { return y[0]; }, Crossing::Rising},
        {"x falling", [](double, const State<2>& y) { return y[0]; }, Crossing::Falling},
    };
    const auto result = integrate<2>(oscillator, 0.0, oscillator_start, 10.0, tight_options(), events);
    REQUIRE(result.event_index == 1);
    CHECK(result.t_s == Catch::Approx(starsim::constants::pi / 2.0).margin(1e-8));
    CHECK(std::abs(result.y[0]) < 1e-8);

    const auto rising = integrate<2>(oscillator, 0.0, oscillator_start, 10.0, tight_options(),
                                     std::span(events).first(1));
    REQUIRE(rising.event_index == 0);
    CHECK(rising.t_s == Catch::Approx(1.5 * starsim::constants::pi).margin(1e-8));
}

TEST_CASE("RK4 converges with 4th order", "[integrators]") {
    auto error_with_step = [](double h) {
        State<2> y = oscillator_start;
        const int steps = static_cast<int>(std::round(2.0 / h));
        for (int i = 0; i < steps; ++i) y = rk4_step(oscillator, i * h, y, h);
        return std::abs(y[0] - std::cos(2.0));
    };
    const double ratio = error_with_step(0.02) / error_with_step(0.01);
    CHECK(ratio == Catch::Approx(16.0).epsilon(0.05));
}
