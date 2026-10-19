// ISS i5: departure guidance after undocking - undock burn, back-out, departure burn 0, drift out of the approach
// ellipsoid (CW point model of proximity_test_support.hpp).
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "proximity_test_support.hpp"
#include "starsim/gnc/departure_guidance.hpp"

using namespace starsim;
using math::Vec3;
using proximity_test::CwSim;

namespace {
struct Flight {
    double kos_s = -1.0, ae_s = -1.0, kg = 0.0;
};
Flight depart(CwSim& s, gnc::DepartureGuidance& g, double limit_s = 7200.0) {
    Flight f;
    const double kg0 = s.propellant_kg;
    for (double t = 0.0; t < limit_s; t += 0.05) {
        s.step(g.update(s.input()).translation_fraction_port, 0.05);
        if (f.kos_s < 0.0 && norm(s.cg.position_m) > 200.0) f.kos_s = t;
        if (g.phase() == gnc::DeparturePhase::Complete) {
            f.ae_s = t;
            break;
        }
    }
    f.kg = kg0 - s.propellant_kg;
    return f;
}
}  // namespace

TEST_CASE("Departure guidance: IDA-2 - undock burn, back-out, departure burn, AE exit within the hour",
          "[iss][docking]") {
    auto s = CwSim::ida2();
    s.cg.position_m = s.port_lever_lvlh + Vec3{4.0, 0, 0};  // chaser CG 4 m out (its port on the station port)
    s.cg.velocity_mps = {0.085, 0.0, 0.0};                    // separation springs
    gnc::DepartureGuidance g({}, {});
    const auto f = depart(s, g);
    REQUIRE(f.ae_s > 0.0);
    CHECK(f.kos_s < 900.0);   // Demo-1: 6 min 48 s
    CHECK(f.ae_s < 3600.0);   // Demo-1: 17 min
    CHECK(f.kg < 60.0);
    REQUIRE_FALSE(g.events().empty());
    CHECK(g.events().front().status == "Abkoppelimpuls");
    REQUIRE(g.departure_burn());
    CHECK(g.departure_burn()->safe);
    if (std::getenv("STARSIM_VERBOSE")) {
        std::printf("[departure] IDA-2: KOS exit %.0f s, AE exit %.0f s, burn %.2f m/s, %.1f kg\n", f.kos_s, f.ae_s,
                    norm(g.departure_burn()->delta_v_lvlh_mps), f.kg);
    }
}

TEST_CASE("Departure guidance: IDA-3 (R-bar) leaves passively safe", "[iss][docking]") {
    auto s = CwSim::ida3();
    s.cg.position_m = s.port_lever_lvlh + Vec3{0, 0, -4.0};
    s.cg.velocity_mps = {0.0, 0.0, -0.085};
    gnc::DepartureGuidance g({}, {});
    const auto f = depart(s, g);
    CHECK(g.phase() == gnc::DeparturePhase::Complete);
    REQUIRE(g.departure_burn());
    CHECK(g.departure_burn()->safe);
    CHECK(f.ae_s < 3600.0);
    if (std::getenv("STARSIM_VERBOSE")) {
        std::printf("[departure] IDA-3: KOS exit %.0f s, AE exit %.0f s, burn %.2f m/s, %.1f kg\n", f.kos_s, f.ae_s,
                    norm(g.departure_burn()->delta_v_lvlh_mps), f.kg);
    }
}

TEST_CASE("Departure guidance: the undock burn reaches 0.15 m/s opening before the back-out", "[iss][docking]") {
    auto s = CwSim::ida2();
    s.cg.position_m = s.port_lever_lvlh + Vec3{4.0, 0, 0};
    s.cg.velocity_mps = {0.085, 0.0, 0.0};
    gnc::DepartureGuidance g({}, {});
    for (double t = 0.0; t < 60.0 && g.phase() == gnc::DeparturePhase::UndockBurn; t += 0.05) {
        s.step(g.update(s.input()).translation_fraction_port, 0.05);
    }
    CHECK(g.phase() == gnc::DeparturePhase::BackOut);
    CHECK(s.input().port.velocity_mps.x == Catch::Approx(0.15).margin(0.01));
}
