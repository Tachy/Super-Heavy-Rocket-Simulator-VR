#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/aero/flap.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using Catch::Approx;

namespace {

constexpr double pi = constants::pi;

aero::FlapTable table() {
    config::CsvTable t;
    t.columns = {"mach", "cn_max"};
    t.rows = {{0.0, 1.2}, {5.0, 1.84}};
    return aero::FlapTable(t);
}

// Belly towards body -z (azimuth 270 deg), flap hinged on the +y side.
aero::FlapGeometry flap() {
    aero::FlapGeometry f;
    f.station_fwd_m = 14.0;
    f.station_aft_m = 2.0;
    f.hinge_azimuth_rad = 0.0;
    f.hull_radius_m = 4.5;
    f.span_m = 4.0;
    f.area_m2 = 48.0;
    f.belly_azimuth_rad = 1.5 * pi;
    return f;
}

const aero::FinAirData air{1e-3, 300.0, 0.0};

}  // namespace

TEST_CASE("Flap geometry: stowed edge-on to a belly-first flow, extended broadside", "[aero][flap]") {
    const auto f = flap();
    const auto s0 = aero::flap_span_direction(f, 0.0);  // folded leeward: +z, away from the belly at -z
    CHECK(s0.z == Approx(1.0).margin(1e-12));
    const auto s90 = aero::flap_span_direction(f, pi / 2);
    CHECK(s90.y == Approx(1.0).margin(1e-12));  // radial at the +y hinge
    CHECK(std::abs(aero::flap_normal(f, pi / 2).z) == Approx(1.0).margin(1e-12));
    const auto c = aero::flap_centroid(f, pi / 2);
    CHECK(c.x == Approx(8.0));
    CHECK(c.y == Approx(6.5));
    CHECK(c.z == Approx(0.0).margin(1e-12));
}

TEST_CASE("Extended flap in a belly-first flow carries Newtonian drag", "[aero][flap]") {
    const math::Vec3 v{0.0, 0.0, -1500.0};  // moving towards the belly (-z), Mach 5
    const double q = 0.5 * 1e-3 * 1500.0 * 1500.0;
    const auto r = aero::flap_force(flap(), pi / 2, v, air, table());
    CHECK(r.force_body_n.z == Approx(1.84 * 48.0 * q));  // opposes the motion
    CHECK(std::abs(r.force_body_n.x) < 1e-6);
    const auto stowed = aero::flap_force(flap(), 0.0, v, air, table());
    CHECK(norm(stowed.force_body_n) < 1e-6 * q);
    const auto half = aero::flap_force(flap(), pi / 4, v, air, table());
    CHECK(norm(half.force_body_n) == Approx(1.84 * 48.0 * q * 0.5));
}

TEST_CASE("Flaps in the hull's lee or in vacuum carry no force", "[aero][flap]") {
    const math::Vec3 back_first{0.0, 0.0, 1500.0};  // moving leeward
    CHECK(norm(aero::flap_force(flap(), pi / 2, back_first, air, table()).force_body_n) == 0.0);
    const aero::FinAirData vacuum{0.0, 0.0, 1.0};
    const auto r = aero::flap_force(flap(), pi / 2, math::Vec3{0.0, 0.0, -7000.0}, vacuum, table());
    CHECK(norm(r.force_body_n) == 0.0);
    CHECK(std::isfinite(r.application_point_m.x));
}

TEST_CASE("Stowed flaps in nose-first flow give no force", "[aero][flap]") {
    const math::Vec3 nose_first{-2000.0, 0.0, 0.0};  // flow along the axis
    CHECK(norm(aero::flap_force(flap(), 0.0, nose_first, air, table()).force_body_n) == 0.0);
}

// Forward flap as drawn: outline centroid in the hinge frame, hinge leaning 19.03 deg.
namespace {
aero::FlapGeometry leaned_flap(double azimuth_rad) {
    aero::FlapGeometry f;
    f.station_aft_m = 42.282;
    f.station_fwd_m = 47.75;
    f.hinge_azimuth_rad = azimuth_rad;
    f.hull_radius_m = 4.5;
    f.span_m = 3.154;
    f.area_m2 = 12.374;
    f.belly_azimuth_rad = 1.5 * pi;
    f.hinge_lean_rad = 19.03 * constants::deg_to_rad;
    f.hinge_radius_aft_m = 4.621;
    f.centroid_u_m = 1.6091;
    f.centroid_v_m = 1.2499;
    return f;
}
}  // namespace

TEST_CASE("Lean 0 with the default centroid is the old straight flap to 1e-12", "[aero][flap]") {
    const math::Vec3 v{-300.0, 200.0, -1500.0};
    for (const double azimuth : {0.0, pi, 20.0 * constants::deg_to_rad, 160.0 * constants::deg_to_rad}) {
        auto f = flap();
        f.hinge_azimuth_rad = azimuth;
        const math::Vec3 e_r{0.0, std::cos(azimuth), std::sin(azimuth)};
        math::Vec3 stow = cross(math::Vec3{1.0, 0.0, 0.0}, e_r);
        if (dot(stow, math::Vec3{0.0, 0.0, -1.0}) > 0.0) stow = stow * -1.0;
        for (const double d : {0.0, 0.3, pi / 4, 1.2, pi / 2}) {
            // The formulas before the lean, written out.
            const math::Vec3 span = stow * std::cos(d) + e_r * std::sin(d);
            const math::Vec3 normal = cross(math::Vec3{1.0, 0.0, 0.0}, span);
            const math::Vec3 point = math::Vec3{0.5 * (f.station_fwd_m + f.station_aft_m), 0.0, 0.0} +
                                     e_r * f.hull_radius_m + span * (0.5 * f.span_m);
            CHECK(norm(aero::flap_span_direction(f, d) - span) < 1e-12);
            CHECK(norm(aero::flap_normal(f, d) - normal) < 1e-12);
            CHECK(norm(aero::flap_centroid(f, d) - point) < 1e-12);
            const auto r = aero::flap_force(f, d, v, air, table());
            CHECK(norm(r.application_point_m - point) < 1e-12);
            const double q = 0.5 * 1e-3 * dot(v, v);
            const double speed = norm(v), vn = dot(v, normal);
            const double cn = table().cn_max(speed / air.speed_of_sound_mps) * (vn / speed) * (vn / speed);
            const math::Vec3 old_force = normal * (-std::copysign(cn * q * f.area_m2, vn));
            CHECK(norm(r.force_body_n - old_force) < 1e-12 * std::max(1.0, norm(old_force)));
        }
    }
}

TEST_CASE("Leaned forward flap: hinge, span and normal stay an orthonormal frame", "[aero][flap]") {
    for (const double azimuth : {20.0 * constants::deg_to_rad, 160.0 * constants::deg_to_rad}) {
        const auto f = leaned_flap(azimuth);
        const math::Vec3 e_r{0.0, std::cos(azimuth), std::sin(azimuth)};
        const auto h = aero::flap_hinge_direction(f);
        CHECK(h.x == Approx(std::cos(f.hinge_lean_rad)));
        CHECK(dot(h, e_r) == Approx(-std::sin(f.hinge_lean_rad)));
        for (const double d : {0.0, 0.7, pi / 2}) {
            const auto s = aero::flap_span_direction(f, d);
            const auto n = aero::flap_normal(f, d);
            CHECK(norm(s) == Approx(1.0));
            CHECK(norm(n) == Approx(1.0));
            CHECK(dot(s, h) == Approx(0.0).margin(1e-12));
            CHECK(dot(n, s) == Approx(0.0).margin(1e-12));
            CHECK(dot(n, h) == Approx(0.0).margin(1e-12));
        }
        // Fully out: in the meridian plane, the normal perpendicular to the body axis.
        CHECK(aero::flap_normal(f, pi / 2).x == Approx(0.0).margin(1e-12));
        CHECK(dot(aero::flap_normal(f, pi / 2), e_r) == Approx(0.0).margin(1e-12));
    }
}

TEST_CASE("Leaned forward flap: the application point is the real planform's centroid", "[aero][flap]") {
    const auto f = leaned_flap(20.0 * constants::deg_to_rad);
    const double lean = f.hinge_lean_rad;
    const math::Vec3 e_r{0.0, std::cos(f.hinge_azimuth_rad), std::sin(f.hinge_azimuth_rad)};
    const auto c = aero::flap_centroid(f, pi / 2);
    CHECK(c.x == Approx(42.282 + 1.6091 * std::cos(lean) + 1.2499 * std::sin(lean)));
    CHECK(dot(c, e_r) == Approx(4.621 - 1.6091 * std::sin(lean) + 1.2499 * std::cos(lean)));
    CHECK(aero::flap_force(f, pi / 2, {0.0, 0.0, -1500.0}, air, table()).application_point_m.x == Approx(c.x));
}

TEST_CASE("Flaps at d = 90 carry no force in axial flow", "[aero][flap]") {
    for (const auto& f : {flap(), leaned_flap(20.0 * constants::deg_to_rad), leaned_flap(160.0 * constants::deg_to_rad)}) {
        for (const double speed : {-2000.0, 2000.0}) {
            CHECK(norm(aero::flap_force(f, pi / 2, {speed, 0.0, 0.0}, air, table()).force_body_n) == 0.0);
        }
        // The lee test does not hide it: a whiff towards the belly gives only the second-order sin^2 force.
        const double crossflow = 1.0, speed = 2000.0, q = 0.5 * 1e-3 * speed * speed;
        const auto r = aero::flap_force(f, pi / 2, {-speed, 0.0, -crossflow}, air, table());
        CHECK(norm(r.force_body_n) < 2.0 * f.area_m2 * q * (crossflow / speed) * (crossflow / speed));
        CHECK(std::abs(r.force_body_n.x) < 1e-9);
    }
}

// ---------------------------------------------------------------- flaps on the vehicle

#include "starsim/environment/atmosphere.hpp"
#include "starsim/vehicle/flight_body.hpp"

namespace {

const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;
const time::Epoch t0 = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);

class ZeroGravity final : public environment::GravityModel {
public:
    ephemeris::Body central_body() const override { return ephemeris::Body::Earth; }
    math::Vec3 acceleration_mps2(const math::Vec3&, time::Epoch) const override { return {}; }
};

class FixedEarth final : public frames::BodyOrientation {
public:
    frames::FixedToInertial fixed_to_gcrf(time::Epoch) const override { return {math::Mat3::identity(), {}}; }
};

struct World {
    ZeroGravity gravity;
    environment::Us76Atmosphere atmosphere;
    FixedEarth earth;
    vehicle::FlightEnvironment env{gravity, atmosphere, earth, nullptr, false};
};

config::VehicleConfig ship_config(double flap_deflection_rad) {
    auto cfg = config::load_vehicle(config_dir, "starship_block3");
    cfg.stages.erase(cfg.stages.begin());
    for (auto& f : cfg.stages[0].flaps) f.initial_deflection_rad = flap_deflection_rad;
    return cfg;
}

// Ship alone at 60 km over the equator, nose up (body x = GCRF x = local up), moving belly first (body -z).
std::unique_ptr<vehicle::FlightBody> belly_first_ship(double flap_deflection_rad, double speed_mps) {
    const vehicle::RigidBodyState state{{6378137.0 + 60000.0, 0.0, 0.0}, {0.0, 0.0, -speed_mps}, {}, {}};
    return std::make_unique<vehicle::FlightBody>(
        std::make_unique<vehicle::Vehicle>(ship_config(flap_deflection_rad), config_dir), state);
}

}  // namespace

TEST_CASE("The block 3 ship has four flaps with rate-limited actuators", "[flap][vehicle]") {
    const auto cfg = config::load_vehicle(config_dir, "starship_block3");
    REQUIRE(cfg.stages.back().flaps.size() == 4);
    CHECK(cfg.stages.front().flaps.empty());
    vehicle::Vehicle v(cfg, config_dir);
    REQUIRE(v.flap_count() == 4);
    vehicle::VehicleCommand c;
    c.flap_rad = {1.5, 1.5, 1.5, 1.5};
    const auto& f = v.stages().back().flaps[0];
    CHECK(f.deflection_rad == Approx(f.max_deflection_rad));  // rest pose: fully out, d = 90 deg
    CHECK(f.max_deflection_rad == Approx(0.5 * pi));
    CHECK(f.rate_rad_s == Approx(20.0 * constants::deg_to_rad));
    c.flap_rad = {0.0, 0.0, 0.0, 0.0};
    v.update_flaps(0.1, c);
    CHECK(f.max_deflection_rad - f.deflection_rad == Approx(f.rate_rad_s * 0.1));
    c.flap_rad = {1.5, 1.5, 1.5, 1.5};
    for (int i = 0; i < 200; ++i) v.update_flaps(0.1, c);
    CHECK(f.deflection_rad == Approx(std::min(1.5, f.max_deflection_rad)));
    c.flap_rad.clear();  // no command: the flaps hold their deflection
    v.update_flaps(0.1, c);
    CHECK(f.deflection_rad == Approx(std::min(1.5, f.max_deflection_rad)));
    c.flap_rad = {-1.0, 9.0};
    for (int i = 0; i < 200; ++i) v.update_flaps(0.1, c);
    CHECK(v.stages().back().flaps[0].deflection_rad == Approx(0.0).margin(1e-12));
    CHECK(v.stages().back().flaps[1].deflection_rad == Approx(v.stages().back().flaps[1].max_deflection_rad));
}

TEST_CASE("Ship flaps rest at d = 90 and follow the drawn geometry", "[flap][vehicle]") {
    // The tanker inherits the block 3 flaps; depot and HLS have none.
    for (const char* id : {"starship_block3", "starship_tanker"}) {
        const auto cfg = config::load_vehicle(config_dir, id);
        REQUIRE(cfg.stages.back().flaps.size() == 4);
        vehicle::Vehicle v(cfg, config_dir);
        for (const auto& f : v.stages().back().flaps) {
            CHECK(f.deflection_rad == Approx(0.5 * pi));
            CHECK(std::abs(aero::flap_normal(f.geometry, f.deflection_rad).x) < 1e-12);
            const bool forward = f.geometry.hinge_lean_rad > 0.0;
            CHECK(f.geometry.area_m2 == Approx(forward ? 12.374 : 36.48).epsilon(1e-3));
            CHECK(f.geometry.span_m == Approx(forward ? 3.154 : 3.69));
            CHECK(f.geometry.station_fwd_m - f.geometry.station_aft_m == Approx(forward ? 5.468 : 12.0).epsilon(1e-3));
            CHECK(f.geometry.hinge_lean_rad == Approx(forward ? 19.03 * constants::deg_to_rad : 0.0));
            CHECK(f.max_deflection_rad == Approx(0.5 * pi));
        }
    }
    for (const char* id : {"starship_hls", "starship_depot"}) {
        CHECK(config::load_vehicle(config_dir, id).stages.back().flaps.empty());
    }
}

TEST_CASE("No command in coast: the flaps hold the rest pose", "[flap][vehicle]") {
    auto cfg = config::load_vehicle(config_dir, "starship_block3");
    vehicle::Vehicle v(cfg, config_dir);
    for (int i = 0; i < 100; ++i) v.update_flaps(0.1, {});
    for (const auto& f : v.stages().back().flaps) CHECK(f.deflection_rad == Approx(0.5 * pi));
}

TEST_CASE("Extended flaps add their Newtonian drag to a belly-first ship", "[flap][vehicle]") {
    World world;
    const double speed = 1500.0;
    auto extended = belly_first_ship(0.5 * pi, speed);
    auto stowed = belly_first_ship(0.0, speed);
    const double dt = 0.005;
    extended->step(world.env, t0, dt, {});
    stowed->step(world.env, t0, dt, {});
    const double extra = (extended->state().velocity_gcrf_mps.z - stowed->state().velocity_gcrf_mps.z) / dt;

    const auto air = world.atmosphere.at(60000.0);
    const double q = 0.5 * air.density_kg_m3 * speed * speed;
    const aero::FlapTable t(config::load_csv_table(config_dir / "aero/flap.csv"));
    const double cn = t.cn_max(speed / air.speed_of_sound_mps);
    // Along the motion each flap gives cn A sin^2(theta) cos(theta'): broadside for hinges on the sides; a hinge
    // phi off the side tilts the normal by phi (sin^2 theta = cos^2 phi, z share cos phi).
    double expected_force = 0.0;
    for (const auto& f : extended->vehicle().stages().back().flaps) {
        const double tilt = std::asin(std::abs(std::sin(f.geometry.hinge_azimuth_rad)));
        expected_force += cn * f.geometry.area_m2 * std::pow(std::cos(tilt), 3);
    }
    const double mass = extended->vehicle().mass_properties().mass_kg;
    CHECK(extra == Approx(expected_force * q / mass).epsilon(0.05));
}
