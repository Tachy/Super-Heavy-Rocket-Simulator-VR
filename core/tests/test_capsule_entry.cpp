// Crew Dragon capsule entry (d3 of docs/superpowers/specs/2026-10-14-rcs-crew-dragon-design.md): the capsule flies its
// blunt-body aerodynamics with an offset CG, its Dracos fly the bank, the entry guidance takes it to the drogue gate.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>

#include "ship_entry_scenario.hpp"
#include "starsim/aero/capsule_aero.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/math/mat3.hpp"
#include "starsim/vehicle/flight_body.hpp"

using namespace starsim;
using math::Vec3;

namespace {

constexpr double deg = constants::deg_to_rad;

// The capsule alone (trunk jettisoned), RCS tank at `propellant_kg`.
config::VehicleConfig capsule_config(double propellant_kg = 2100.0) {
    auto c = config::load_vehicle(STARSIM_CONFIG_DIR, "crew_dragon");
    c.stages.erase(c.stages.begin());
    for (auto& t : c.stages.front().tanks) t.mass_kg = propellant_kg;
    return c;
}

// Body -> GCRF rotation that maps the body pair (a, b) onto the GCRF pair (c, d) (b, d made orthogonal to a, c).
math::Quat attitude_from_pairs(const Vec3& a_body, const Vec3& b_body, const Vec3& c_gcrf, const Vec3& d_gcrf) {
    const auto triad = [](const Vec3& a, const Vec3& b) {
        const Vec3 e1 = math::normalized(a);
        const Vec3 e2 = math::normalized(b - e1 * dot(b, e1));
        const Vec3 e3 = cross(e1, e2);
        math::Mat3 m;
        const Vec3 cols[3] = {e1, e2, e3};
        for (int c = 0; c < 3; ++c) {
            m(0, c) = cols[c].x;
            m(1, c) = cols[c].y;
            m(2, c) = cols[c].z;
        }
        return m;
    };
    return math::Quat::from_matrix(triad(c_gcrf, d_gcrf) * transpose(triad(a_body, b_body))).normalized();
}

// The capsule alone over the test Earth of ship_entry_scenario.hpp (central gravity, rotating, US 1976 air).
class CapsuleEntryScenario {
public:
    // `theta_offset_rad`: start this far beyond the trim angle (in the trim plane).
    CapsuleEntryScenario(double height_m, double speed_relative_mps, double flight_path_rad, double theta_offset_rad = 0.0,
                         double propellant_kg = 2100.0) {
        auto vehicle = std::make_unique<vehicle::Vehicle>(capsule_config(propellant_kg), STARSIM_CONFIG_DIR);
        const auto trim = vehicle->capsule_aero()->trim(20.0, vehicle->mass_properties().cg_point_m());
        const frames::GeodeticPosition g{10.0 * deg, 0.0, height_m};
        const Vec3 r = frames::geodetic_to_fixed(g);
        const Vec3 up = math::normalized(r);
        const Vec3 east{0.0, 1.0, 0.0};
        const Vec3 v_rel = (east * std::cos(flight_path_rad) + up * std::sin(flight_path_rad)) * speed_relative_mps;
        const Vec3 v = v_rel + cross(Vec3{0.0, 0.0, test::earth_rate_radps}, r);
        // trim plane x-rho in the body, rho the CG offset direction; u_body at theta = trim + offset
        const Vec3 rho = math::normalized(trim.air_velocity_body_unit - Vec3{trim.air_velocity_body_unit.x, 0.0, 0.0});
        const double theta = trim.theta_rad + theta_offset_rad;
        const Vec3 u_body = Vec3{-std::cos(theta), 0.0, 0.0} + rho * std::sin(theta);
        const Vec3 normal_body = cross(Vec3{1.0, 0.0, 0.0}, rho);  // perpendicular to the trim plane
        const Vec3 normal_gcrf = math::normalized(cross(v_rel, up));
        const vehicle::RigidBodyState state{r, v, attitude_from_pairs(u_body, normal_body, v_rel, normal_gcrf), {}};
        trim_ = trim;
        body_ = std::make_unique<vehicle::FlightBody>(std::move(vehicle), state);
    }

    vehicle::FlightBody& body() { return *body_; }
    const vehicle::FlightEnvironment& environment() const { return env_; }
    const aero::CapsuleTrim& trim() const { return trim_; }
    time::Epoch epoch() const { return test::entry_t0 + time_s_; }
    double time_s() const { return time_s_; }

    Vec3 air_velocity_body() const {
        const auto t = env_.earth_orientation.fixed_to_gcrf(epoch());
        const auto& s = body_->state();
        const Vec3 air = s.velocity_gcrf_mps - cross(vehicle::earth_rotation_vector(t), s.position_gcrf_m);
        return s.attitude.inverse_rotate(air);
    }
    double theta_rad() const {
        const Vec3 u = air_velocity_body();
        return std::acos(std::clamp(-u.x / norm(u), -1.0, 1.0));
    }
    double altitude_m() const {
        const auto t = env_.earth_orientation.fixed_to_gcrf(epoch());
        return frames::fixed_to_geodetic(transpose(t.rotation) * body_->state().position_gcrf_m).height_m;
    }

    void step(const vehicle::VehicleCommand& command = {}) {
        body_->step(env_, epoch(), dt_s, command);
        time_s_ += dt_s;
    }

    static constexpr double dt_s = 0.005;

private:
    test::EntryGravity gravity_;
    test::EntryEarth earth_;
    environment::Us76Atmosphere atmosphere_;
    vehicle::FlightEnvironment env_{gravity_, atmosphere_, earth_, nullptr, false};
    std::unique_ptr<vehicle::FlightBody> body_;
    aero::CapsuleTrim trim_;
    double time_s_ = 0.0;
};

}  // namespace

TEST_CASE("Crew Dragon capsule alone flies the capsule aerodynamics with its CG offset", "[dragon][entry]") {
    vehicle::Vehicle capsule(capsule_config(), STARSIM_CONFIG_DIR);
    REQUIRE(capsule.capsule_aero() != nullptr);
    const auto mp = capsule.mass_properties();
    CHECK(std::abs(mp.cg_offset_m.y) + std::abs(mp.cg_offset_m.z) > 0.01);
    CHECK(mp.cg_offset_m.x == 0.0);
    const auto t = capsule.capsule_aero()->trim(20.0, mp.cg_point_m());
    REQUIRE(t.found);
    std::printf("[validation] Dragon capsule %.0f kg, CG station %.3f m, offset %.4f m: trim theta %.2f deg, L/D %.3f, "
                "C_D %.3f\n",
                mp.mass_kg, mp.cg_station_m, mp.cg_offset_m.y, t.theta_rad / deg, t.lift_to_drag, t.cd);
    CHECK(t.lift_to_drag == Catch::Approx(0.20).margin(0.03));
    // with the trunk the stack still flies the slender body (orbit, deorbit burn)
    vehicle::Vehicle stack(config::load_vehicle(STARSIM_CONFIG_DIR, "crew_dragon"), STARSIM_CONFIG_DIR);
    CHECK(stack.capsule_aero() == nullptr);
    // ... and gets it once the trunk is jettisoned
    auto upper = stack.separate(1);
    CHECK(upper->capsule_aero() != nullptr);
    CHECK(stack.capsule_aero() == nullptr);
}

TEST_CASE("An uncontrolled capsule in hypersonic flow oscillates about its trim angle", "[dragon][entry]") {
    // 60 km, 7 km/s level, started 10 deg beyond trim; no RCS and no aerodynamic damping: it oscillates about the trim.
    CapsuleEntryScenario s(60000.0, 7000.0, 0.0, 10.0 * deg);
    double sum = 0.0, lo = 1e9, hi = -1e9;
    int n = 0;
    while (s.time_s() < 60.0) {
        s.step();
        if (s.time_s() > 40.0) {
            const double th = s.theta_rad();
            sum += th;
            lo = std::min(lo, th);
            hi = std::max(hi, th);
            ++n;
        }
    }
    const double mean = sum / n;
    std::printf("[validation] uncontrolled capsule: theta %.2f..%.2f deg (mean %.2f), trim %.2f deg\n", lo / deg, hi / deg,
                mean / deg, s.trim().theta_rad / deg);
    CHECK(std::abs(mean - s.trim().theta_rad) < 2.0 * deg);
    CHECK(hi - lo < 25.0 * deg);  // bounded (statically stable)
    CHECK_FALSE(s.body().impacted());
}

TEST_CASE("Vehicles without a lateral CG offset keep their moments", "[vehicle][rcs]") {
    vehicle::Vehicle ship(config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3"), STARSIM_CONFIG_DIR);
    CHECK(ship.mass_properties().cg_offset_m.y == 0.0);
    CHECK(ship.mass_properties().cg_offset_m.z == 0.0);
    CHECK(ship.capsule_aero() == nullptr);
    vehicle::Vehicle f9(config::load_vehicle(STARSIM_CONFIG_DIR, "falcon9_block5"), STARSIM_CONFIG_DIR);
    CHECK(f9.mass_properties().cg_point_m().y == 0.0);
}
