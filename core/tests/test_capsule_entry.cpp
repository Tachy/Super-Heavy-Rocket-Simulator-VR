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
#include "starsim/config/splashdown_zone_config.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/gnc/entry_aero_table.hpp"
#include "starsim/gnc/entry_predictor.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"
#include "starsim/gnc/capsule_entry_control.hpp"
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
    Vec3 air_velocity_gcrf() const { return body_->state().attitude.rotate(air_velocity_body()); }
    Vec3 up_gcrf() const { return math::normalized(body_->state().position_gcrf_m); }
    double dynamic_pressure_pa() const {
        const double v = norm(air_velocity_body());
        return 0.5 * atmosphere_.at(altitude_m()).density_kg_m3 * v * v;
    }
    double bank_rad() const { return gnc::capsule_bank_rad(*body_, env_, epoch()); }
    // One tick with the Dracos flying `bank_command_rad`.
    void fly(double bank_command_rad) {
        vehicle::VehicleCommand c;
        c.rcs_torque_fraction = gnc::capsule_entry_torque_fraction(*body_, env_, epoch(), bank_command_rad);
        step(c);
    }
    void set_bank(double bank_rad) {
        auto st = body_->state();
        st.attitude = gnc::capsule_trim_attitude(body_->vehicle(), air_velocity_gcrf(), up_gcrf(), bank_rad);
        st.angular_velocity_body_radps = {};
        body_->set_state(st);
    }
    Vec3 position_itrf_m() const {
        return transpose(env_.earth_orientation.fixed_to_gcrf(epoch()).rotation) * body_->state().position_gcrf_m;
    }
    Vec3 velocity_itrf_mps() const {
        const auto t = env_.earth_orientation.fixed_to_gcrf(epoch());
        const auto& st = body_->state();
        return transpose(t.rotation) * (st.velocity_gcrf_mps - cross(vehicle::earth_rotation_vector(t), st.position_gcrf_m));
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

TEST_CASE("Capsule trim attitude has the commanded bank", "[dragon][entry]") {
    CapsuleEntryScenario s(60000.0, 7000.0, -1.0 * deg);
    for (const double bank_deg : {-120.0, -30.0, 0.0, 45.0, 150.0}) {
        s.set_bank(bank_deg * deg);
        CHECK(s.bank_rad() / deg == Catch::Approx(bank_deg).margin(0.5));
        CHECK(s.theta_rad() == Catch::Approx(s.trim().theta_rad).margin(0.2 * deg));
    }
}

TEST_CASE("The Dracos roll the capsule to a commanded bank and hold the trim", "[dragon][entry]") {
    CapsuleEntryScenario s(60000.0, 7000.0, -1.0 * deg);
    s.set_bank(0.0);
    const double propellant0 = s.body().vehicle().rcs_propellant_kg();
    while (s.time_s() < 15.0) s.fly(60.0 * deg);
    const auto w = s.body().state().angular_velocity_body_radps;
    const Vec3 u = math::normalized(s.air_velocity_body());
    const Vec3 w_perp = w - u * dot(w, u);
    std::printf("[validation] capsule bank 0 -> 60 deg: %.1f deg after 15 s, theta %.1f deg (trim %.1f), pitch/yaw rate "
                "%.2f deg/s\n",
                s.bank_rad() / deg, s.theta_rad() / deg, s.trim().theta_rad / deg, norm(w_perp) / deg);
    CHECK(std::abs(s.bank_rad() - 60.0 * deg) < 5.0 * deg);
    CHECK(std::abs(s.theta_rad() - s.trim().theta_rad) < 3.0 * deg);
    CHECK(norm(w_perp) < 2.0 * deg);
    // reversal: through lift up (|bank| < 90 deg on the way), never through lift down
    double worst = 0.0;
    while (s.time_s() < 35.0) {
        s.fly(-60.0 * deg);
        worst = std::max(worst, std::abs(s.bank_rad()));
    }
    std::printf("[validation] capsule reversal 60 -> -60 deg: %.1f deg after 20 s, largest |bank| %.1f deg\n",
                s.bank_rad() / deg, worst / deg);
    CHECK(worst < 90.0 * deg);
    CHECK(std::abs(s.bank_rad() + 60.0 * deg) < 5.0 * deg);
    // holding and switching the bank through the entry costs tens of kilograms (Apollo: ~30-40 kg per entry [E])
    double command = -60.0 * deg;
    while (s.time_s() < 335.0) {
        if (std::fmod(s.time_s(), 60.0) < CapsuleEntryScenario::dt_s) command = -command;
        s.fly(command);
    }
    const double used = propellant0 - s.body().vehicle().rcs_propellant_kg();
    std::printf("[validation] capsule entry control: %.1f kg of Draco propellant in 335 s\n", used);
    CHECK(used < 60.0);
    CHECK_FALSE(s.body().impacted());
}

TEST_CASE("A capsule tumbling off trim in thin air turns to its trim before the air bites", "[dragon][entry]") {
    CapsuleEntryScenario s(110000.0, 7400.0, -1.5 * deg, 0.0);
    {  // 90 deg off the trim, tumbling at 2 deg/s
        CapsuleEntryScenario probe(110000.0, 7400.0, -1.5 * deg, 90.0 * deg - s.trim().theta_rad);
        auto st = probe.body().state();
        st.angular_velocity_body_radps = {0.0, 0.0, 2.0 * deg};
        s.body().set_state(st);
    }
    while (s.dynamic_pressure_pa() < 1000.0 && s.time_s() < 900.0) s.fly(0.0);
    std::printf("[validation] capsule from 90 deg / 2 deg/s at 110 km: at q = 1 kPa (%.0f s, %.1f km) theta %.1f deg "
                "(trim %.1f), bank %.1f deg\n",
                s.time_s(), s.altitude_m() / 1000.0, s.theta_rad() / deg, s.trim().theta_rad / deg, s.bank_rad() / deg);
    REQUIRE(s.dynamic_pressure_pa() >= 1000.0);
    CHECK(std::abs(s.theta_rad() - s.trim().theta_rad) < 5.0 * deg);
    CHECK(std::abs(s.bank_rad()) < 10.0 * deg);
}

namespace {

const spice::SpiceEphemeris& spice_ephemeris() {
    test::spice_session();
    static const spice::SpiceEphemeris e;
    return e;
}

const frames::BodyOrientation& spice_earth() {
    test::spice_session();
    static const auto e = spice::SpiceOrientation::earth();
    return e;
}

// Entry interface state `range_m` uprange of the zone on a great circle that crosses the zone heading `azimuth_rad`
// (clockwise from north), 120 km up, `speed_mps` relative to the Earth at `gamma_rad`; the capsule at its trim with
// `bank_rad`.
vehicle::RigidBodyState entry_interface_state(const config::SplashdownZoneConfig& zone, double range_m, double azimuth_rad,
                                              double speed_mps, double gamma_rad, double bank_rad, time::Epoch epoch,
                                              const vehicle::Vehicle& capsule) {
    const Vec3 z = frames::geodetic_to_fixed({zone.latitude_rad, zone.longitude_rad, 0.0});
    const Vec3 up_z = math::normalized(z);
    const Vec3 east{-std::sin(zone.longitude_rad), std::cos(zone.longitude_rad), 0.0};
    const Vec3 north = cross(up_z, east);
    const Vec3 track = north * std::cos(azimuth_rad) + east * std::sin(azimuth_rad);  // direction of flight at the zone
    const double a = range_m / constants::wgs84_semi_major_axis_m;
    const Vec3 dir = up_z * std::cos(a) - track * std::sin(a);                        // start direction (uprange)
    const Vec3 heading = math::normalized(up_z * std::sin(a) + track * std::cos(a));  // towards the zone
    const auto g = frames::fixed_to_geodetic(dir * constants::wgs84_semi_major_axis_m);
    const Vec3 r = frames::geodetic_to_fixed({g.latitude_rad, g.longitude_rad, 120000.0});
    const Vec3 up = math::normalized(r);
    const Vec3 h = math::normalized(heading - up * dot(heading, up));
    const Vec3 v_rel = (h * std::cos(gamma_rad) + up * std::sin(gamma_rad)) * speed_mps;
    const auto t = spice_earth().fixed_to_gcrf(epoch);
    const Vec3 r_gcrf = t.rotation * r;
    const Vec3 v_rel_gcrf = t.rotation * v_rel;
    vehicle::RigidBodyState s;
    s.position_gcrf_m = r_gcrf;
    s.velocity_gcrf_mps = v_rel_gcrf + cross(vehicle::earth_rotation_vector(t), r_gcrf);
    s.attitude = gnc::capsule_trim_attitude(capsule, v_rel_gcrf, math::normalized(r_gcrf), bank_rad);
    return s;
}

double horizontal_miss_m(const Vec3& r_itrf, const Vec3& target_itrf) {
    const Vec3 up = math::normalized(r_itrf);
    const Vec3 d = r_itrf - target_itrf;
    return norm(d - up * dot(d, up));
}

// Range of the nominal entry (predictor at `bank_rad`) so that a start that far uprange ends at the zone.
double nominal_entry_range_m(const config::SplashdownZoneConfig& zone, double azimuth_rad, double speed_mps,
                             double gamma_rad, double bank_rad, time::Epoch t0, const vehicle::Vehicle& capsule) {
    double range = 2000000.0;
    for (int i = 0; i < 4; ++i) {
        const auto s0 = entry_interface_state(zone, range, azimuth_rad, speed_mps, gamma_rad, bank_rad, t0, capsule);
        gnc::EntryPredictorModel model;
        model.aero = gnc::EntryAeroTable::capsule(capsule);
        model.mass_kg = capsule.mass_properties().mass_kg;
        const auto t = spice_earth().fixed_to_gcrf(t0);
        gnc::EntryPredictorState start;
        start.position_itrf_m = transpose(t.rotation) * s0.position_gcrf_m;
        start.velocity_itrf_mps =
            transpose(t.rotation) * (s0.velocity_gcrf_mps - cross(vehicle::earth_rotation_vector(t), s0.position_gcrf_m));
        start.bank_rad = bank_rad;
        gnc::EntryTarget target;
        target.position_itrf_m = frames::geodetic_to_fixed({zone.latitude_rad, zone.longitude_rad, 0.0});
        const auto p = gnc::predict_entry(model, start, bank_rad, target, 5500.0);
        range += p.downrange_error_m;  // overshoot: start farther uprange
    }
    return range;
}

struct GateResult {
    double time_s = 0.0;
    double miss_m = 0.0;
    double speed_mps = 0.0;
    double altitude_m = 0.0;
    double peak_g = 0.0;
    bool reached = false;
};

// Flies the session's capsule (body 0) to its gate, or impact, or `max_s`.
GateResult fly_capsule_to_gate(sim::FlightSession& session, time::Epoch t0, double max_s = 3000.0) {
    GateResult g;
    double time_s = 0.0;
    while (session.entry_guidance(0)->phase() != gnc::EntryPhase::Gate && !session.body(0).impacted() && time_s < max_s) {
        time_s += session.telemetry(0).altitude_m < 8000.0 ? 0.05 : 1.0;
        session.advance_to(t0 + time_s);
        g.peak_g = std::max(g.peak_g, norm(session.body(0).aero_acceleration_body_mps2()) / constants::standard_gravity_mps2);
        if (std::getenv("STARSIM_VERBOSE") && static_cast<int>(time_s) % 20 == 0 && time_s == std::floor(time_s)) {
            const auto* st = session.entry_status(0);
            const auto tel = session.telemetry(0);
            std::printf("t %.0f h %.0f v %.0f Mach %.2f bank %.1f (flown %.1f) downrange %.0f cross %.0f scales %.3f %.3f\n",
                        time_s, tel.altitude_m, tel.speed_relative_mps, tel.mach, st->bank_rad / deg,
                        gnc::capsule_bank_rad(session.body(0), session.environment(), session.epoch()) / deg,
                        st->downrange_error_m, st->crossrange_m, st->drag_scale, st->lift_scale);
        }
    }
    const auto tr = spice_earth().fixed_to_gcrf(session.epoch());
    const Vec3 r = transpose(tr.rotation) * session.body(0).state().position_gcrf_m;
    const auto tel = session.telemetry(0);
    g.time_s = time_s;
    g.miss_m = horizontal_miss_m(r, session.entry_guidance(0)->target().position_itrf_m);
    g.speed_mps = tel.speed_relative_mps;
    g.altitude_m = tel.altitude_m;
    g.reached = session.entry_guidance(0)->phase() == gnc::EntryPhase::Gate;
    return g;
}

}  // namespace

TEST_CASE("Splashdown zone loads", "[dragon][config]") {
    const auto z = config::load_splashdown_zone(STARSIM_CONFIG_DIR, "pacific_san_diego");
    CHECK(z.latitude_rad / deg == Catch::Approx(32.60).margin(1e-9));
    CHECK(z.longitude_rad / deg == Catch::Approx(-117.80).margin(1e-9));
    CHECK(z.radius_m == 10000.0);
    CHECK_THROWS(config::load_splashdown_zone(STARSIM_CONFIG_DIR, "atlantis"));
}

TEST_CASE("Capsule entry table follows the trim", "[dragon][entry]") {
    vehicle::Vehicle capsule(capsule_config(), STARSIM_CONFIG_DIR);
    const auto table = gnc::EntryAeroTable::capsule(capsule);
    const auto trim = capsule.capsule_aero()->trim(20.0, capsule.mass_properties().cg_point_m());
    CHECK(table.cl_area_m2(20.0) / table.cd_area_m2(20.0) == Catch::Approx(trim.lift_to_drag).epsilon(1e-9));
    CHECK(table.cd_area_m2(20.0) == Catch::Approx(trim.cd * capsule.capsule_aero()->reference_area_m2()).epsilon(1e-9));
    CHECK(table.alpha_rad(20.0) == Catch::Approx(constants::pi - trim.theta_rad).epsilon(1e-9));
    CHECK(table.cd_area_m2(20.0, 1.0) == Catch::Approx(table.cd_area_m2(20.0)).epsilon(1e-9));
}

TEST_CASE("The capsule predictor matches the flown entry at constant bank", "[dragon][entry]") {
    CapsuleEntryScenario s(120000.0, 7400.0, -1.2 * deg);
    s.set_bank(60.0 * deg);
    gnc::EntryPredictorModel model;
    model.aero = gnc::EntryAeroTable::capsule(s.body().vehicle());
    model.mass_kg = s.body().vehicle().mass_properties().mass_kg;
    model.j2 = 0.0;
    model.gm_m3ps2 = 3.986004418e14;  // test::EntryGravity
    model.omega_radps = test::earth_rate_radps;
    gnc::EntryPredictorState start;
    start.position_itrf_m = s.position_itrf_m();
    start.velocity_itrf_mps = s.velocity_itrf_mps();
    start.bank_rad = 60.0 * deg;
    gnc::EntryTarget target;
    target.position_itrf_m = math::normalized(s.position_itrf_m()) * constants::wgs84_semi_major_axis_m;
    const auto p = gnc::predict_entry(model, start, 60.0 * deg, target, 5500.0, 0.0, 0.5);
    REQUIRE(p.reached_gate);
    while (s.altitude_m() > 5500.0 && s.time_s() < 3000.0) s.fly(60.0 * deg);
    const double miss = gnc::ground_range_m(p.end_position_itrf_m, s.position_itrf_m());
    std::printf("[validation] capsule predictor vs flown at 60 deg bank: %.1f km apart, %.0f vs %.0f s, end speed %.0f vs "
                "%.0f m/s\n",
                miss / 1000.0, p.time_s, s.time_s(), norm(p.end_velocity_itrf_mps), norm(s.velocity_itrf_mps()));
    CHECK(miss < 15000.0);
    CHECK(std::abs(p.time_s - s.time_s()) < 20.0);
}

TEST_CASE("Capsule entry guidance reaches the drogue gate", "[dragon][entry][session]") {
    test::spice_session();
    const auto t0 = spice::SpiceTimeConverter{}.from_utc({2026, 10, 14, 18, 0, 0.0});
    const auto zone = config::load_splashdown_zone(STARSIM_CONFIG_DIR, "pacific_san_diego");
    const auto cfg = capsule_config();
    const vehicle::Vehicle capsule(cfg, STARSIM_CONFIG_DIR);
    // the nominal entry of the deorbit plan: -2.0 deg at the interface, 30 deg of bank (d3 ledger: under the crew's 4 g
    // an L/D 0.2 capsule has range authority only between ~0 and ~45 deg of bank)
    const double azimuth = 120.0 * deg, speed = 7500.0, gamma = -2.0 * deg;  // arriving from the west-north-west
    const double range = nominal_entry_range_m(zone, azimuth, speed, gamma, 30.0 * deg, t0, capsule);
    sim::FlightSession session(spice_ephemeris(), spice_earth(), STARSIM_CONFIG_DIR);
    session.set_splashdown_zone(zone);
    session.place_entering_capsule(cfg, entry_interface_state(zone, range, azimuth, speed, gamma, 30.0 * deg, t0, capsule),
                                   t0);
    const auto g = fly_capsule_to_gate(session, t0);
    std::printf("[validation] capsule entry from %.0f km uprange: gate after %.0f s at %.0f m, miss %.2f km, %.0f m/s "
                "(%.0f km/h), peak %.2f g, heat %.0f kW/m2 / %.0f MJ/m2, crew warning %d\n",
                range / 1000.0, g.time_s, g.altitude_m, g.miss_m / 1000.0, g.speed_mps, g.speed_mps * 3.6, g.peak_g,
                session.body(0).peak_heat_rate_w_m2() / 1000.0, session.body(0).heat_load_j_m2() / 1e6,
                session.body(0).crew_load_warning());
    CHECK(g.reached);
    CHECK_FALSE(session.body(0).impacted());
    CHECK(g.miss_m < 2000.0);
    CHECK(g.speed_mps == Catch::Approx(156.0).epsilon(0.15));  // ~350 mph at ~18,000 ft [S: NASA Crew-2]
    CHECK(g.peak_g < 4.5);
}

