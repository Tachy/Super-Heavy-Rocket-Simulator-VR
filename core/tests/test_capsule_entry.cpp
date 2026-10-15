// Crew Dragon capsule entry (d3 of docs/superpowers/specs/2026-10-14-rcs-crew-dragon-design.md): the capsule flies its
// blunt-body aerodynamics with an offset CG, its Dracos fly the bank, the entry guidance takes it to the drogue gate.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <random>

#include "ship_entry_scenario.hpp"
#include "starsim/aero/capsule_aero.hpp"
#include "starsim/config/splashdown_zone_config.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/gnc/entry_aero_table.hpp"
#include "starsim/gnc/deorbit_planner.hpp"
#include "starsim/orbit/ballistic_propagator.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/gnc/entry_predictor.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/sim/landing_outcome.hpp"
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
                                              const vehicle::Vehicle& capsule, double lateral_offset_m = 0.0) {
    const Vec3 z = frames::geodetic_to_fixed({zone.latitude_rad, zone.longitude_rad, 0.0});
    const Vec3 east{-std::sin(zone.longitude_rad), std::cos(zone.longitude_rad), 0.0};
    const Vec3 north0 = cross(math::normalized(z), east);
    const Vec3 track0 = north0 * std::cos(azimuth_rad) + east * std::sin(azimuth_rad);
    // the great circle passes `lateral_offset_m` to the right of the zone (> 0)
    const double o = lateral_offset_m / constants::wgs84_semi_major_axis_m;
    const Vec3 right = cross(track0, math::normalized(z));
    const Vec3 up_z = math::normalized(math::normalized(z) * std::cos(o) + right * std::sin(o));
    const Vec3 track = math::normalized(track0 - up_z * dot(track0, up_z));  // direction of flight abeam the zone
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

// The nominal entry (predictor at +-`bank_rad`, both signs averaged as the deorbit planner does): the range uprange of
// the zone and the lateral offset of the arrival great circle (in/out: start value) whose start ends at the zone along
// and across the track - what a planned deorbit delivers.
double nominal_entry_range_m(const config::SplashdownZoneConfig& zone, double& lateral_offset_m, double azimuth_rad,
                             double speed_mps, double gamma_rad, double bank_rad, time::Epoch t0,
                             const vehicle::Vehicle& capsule) {
    const auto errors = [&](double range, double offset, double& downrange, double& crossrange) {
        const auto s0 = entry_interface_state(zone, range, azimuth_rad, speed_mps, gamma_rad, bank_rad, t0, capsule, offset);
        gnc::EntryPredictorModel model;
        model.aero = gnc::EntryAeroTable::capsule(capsule);
        model.mass_kg = capsule.mass_properties().mass_kg;
        const auto t = spice_earth().fixed_to_gcrf(t0);
        gnc::EntryTarget target;
        target.position_itrf_m = frames::geodetic_to_fixed({zone.latitude_rad, zone.longitude_rad, 0.0});
        downrange = crossrange = 0.0;
        for (const double sign : {1.0, -1.0}) {
            gnc::EntryPredictorState start;
            start.position_itrf_m = transpose(t.rotation) * s0.position_gcrf_m;
            start.velocity_itrf_mps =
                transpose(t.rotation) * (s0.velocity_gcrf_mps - cross(vehicle::earth_rotation_vector(t), s0.position_gcrf_m));
            start.bank_rad = sign * bank_rad;
            const auto p = gnc::predict_entry(model, start, start.bank_rad, target, 5500.0);
            downrange += 0.5 * p.downrange_error_m;
            crossrange += 0.5 * p.crossrange_m;
        }
    };
    double range = 2000000.0;
    for (int i = 0; i < 6; ++i) {
        double d = 0.0, c = 0.0, d2 = 0.0, c2 = 0.0;
        errors(range, lateral_offset_m, d, c);
        range += d;  // overshoot: start farther uprange
        constexpr double step = 1000.0;
        errors(range, lateral_offset_m + step, d2, c2);
        errors(range, lateral_offset_m, d, c);
        if (std::getenv("STARSIM_DEBUG_NOMINAL")) {
            std::printf("nominal %d: range %.1f km offset %.2f km: down %.2f km cross %.2f km (dc/doffset %.3f)\n", i,
                        range / 1000.0, lateral_offset_m / 1000.0, d / 1000.0, c / 1000.0, (c2 - c) / step);
        }
        if (std::abs(c2 - c) > 1e-6) lateral_offset_m -= c * step / (c2 - c);
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
    CHECK(z.radius_m == 2000.0);  // user 2026-10-14
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
    const double azimuth = 120.0 * deg;  // arriving from the west-north-west
    const double speed = 7500.0, gamma = -2.0 * deg;
    double offset = 0.0;
    const double range = nominal_entry_range_m(zone, offset, azimuth, speed, gamma, 30.0 * deg, t0, capsule);
    sim::FlightSession session(spice_ephemeris(), spice_earth(), STARSIM_CONFIG_DIR);
    session.set_splashdown_zone(zone);
    session.place_entering_capsule(
        cfg, entry_interface_state(zone, range, azimuth, speed, gamma, 30.0 * deg, t0, capsule, offset), t0);
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


namespace {

// The Crew Dragon (capsule + trunk) on a 420 km circular orbit at 51.6 deg (ISS-like), nose cone open, splashdown zone set.
struct DragonDeorbit {
    sim::FlightSession session;
    time::Epoch t0;
    config::SplashdownZoneConfig zone;
    // `raan_rad`: the orbit plane; dragon_raan_rad() gives one with a pass over the zone in reach.
    explicit DragonDeorbit(double raan_rad, double inclination_rad = 51.6 * deg)
        : session(spice_ephemeris(), spice_earth(), STARSIM_CONFIG_DIR),
          t0(spice::SpiceTimeConverter{}.from_utc({2026, 10, 14, 6, 0, 0.0})),
          zone(config::load_splashdown_zone(STARSIM_CONFIG_DIR, "pacific_san_diego")) {
        const orbit::KeplerianElements circular{constants::wgs84_semi_major_axis_m + 420e3, 0.0, inclination_rad, raan_rad,
                                                0.0, 0.0};
        session.place_in_orbit("crew_dragon", orbit::state_from_elements(circular, session.gm_earth_m3ps2()), t0);
        session.set_nose_cone(0, true);
        session.set_splashdown_zone(zone);
    }
    // Entry model of the capsule alone after the trunk jettison, with its mass then.
    gnc::EntryPredictorModel capsule_model(double mass_kg) const {
        auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "crew_dragon");
        cfg.stages.erase(cfg.stages.begin());
        const vehicle::Vehicle capsule(cfg, STARSIM_CONFIG_DIR);
        gnc::EntryPredictorModel m;
        m.aero = gnc::EntryAeroTable::capsule(capsule);
        m.mass_kg = mass_kg;
        return m;
    }
    gnc::DeorbitParameters parameters(double thrust_n) const {
        gnc::DeorbitParameters p;
        p.entry_flight_path_rad = -2.0 * deg;
        p.nominal_bank_rad = 30.0 * deg;
        p.max_crossrange_m = 40000.0;
        p.gate_height_m = 5500.0;
        p.burn_thrust_n = thrust_n;
        return p;
    }
    gnc::DeorbitPlan plan(const gnc::DeorbitParameters& p, double entry_mass_kg) {
        const auto& body = session.body(0);
        const auto& s = body.state();
        const auto rcs = body.vehicle().rcs_force_capability({1.0, 0.0, 0.0});
        return gnc::plan_deorbit({s.position_gcrf_m, s.velocity_gcrf_mps}, session.epoch(), session.environment().gravity,
                                 spice_earth(), capsule_model(entry_mass_kg), session.capsule_entry_target(),
                                 body.vehicle().mass_properties().mass_kg, rcs.force_n / rcs.mass_flow_kgps,
                                 body.vehicle().rcs_propellant_kg(), 50.0, p);
    }
    // Flies the node on the Dracos (+x), then coasts to the entry interface; returns the relative flight-path angle there.
    double fly_to_interface(const gnc::DeorbitPlan& plan, double& epoch_error_s, double& burn_s) {
        REQUIRE(session.execute_burn(plan.node));
        double ignition = -1.0, cutoff = -1.0;
        while (session.autopilot().state == sim::AutopilotState::Burn) {
            // long coast to the burn in big steps (a 1 s step is never coasted analytically: ticks)
            const double to_ignition = session.autopilot().burn.time_to_ignition_s;
            session.advance_to(session.epoch() + (to_ignition > 900.0 ? to_ignition - 600.0 : 1.0), 1000000);
            if (ignition < 0.0 && session.autopilot().burn.ignite) ignition = session.epoch() - t0;
        }
        cutoff = session.epoch() - t0;
        burn_s = cutoff - ignition;
        while (session.telemetry(0).altitude_m > 120000.0 && session.epoch() - t0 < 3.0 * 86400.0) {
            const double h = session.telemetry(0).altitude_m;
            session.advance_to(session.epoch() + (h > 130000.0 ? 10.0 : 0.05), 100000);
        }
        epoch_error_s = session.epoch() - plan.entry_interface_epoch;
        return session.telemetry(0).flight_path_angle_rad;
    }
};

// The first orbit plane (RAAN in 10 deg steps) whose impulsive capsule deorbit plan is feasible: a pass over the zone
// within the capsule's crossrange inside the 48 h window (an ISS-like orbit overflies a given point within ~40 km only
// on some days).
double dragon_raan_rad() {
    static const double raan = [] {
        for (double r = 0.0; r < 360.0; r += 10.0) {
            DragonDeorbit d(r * deg);
            const double mass = d.session.body(0).vehicle().mass_properties().mass_kg;
            if (d.plan(d.parameters(0.0), mass - 2000.0).feasible) return r * deg;
        }
        return -1.0;
    }();
    return raan;
}

}  // namespace

TEST_CASE("A finite deorbit burn ends where the planner predicts", "[dragon][deorbit]") {
    REQUIRE(dragon_raan_rad() >= 0.0);
    std::printf("[validation] Dragon orbit plane with a pass in reach: RAAN %.0f deg\n", dragon_raan_rad() / deg);
    DragonDeorbit d(dragon_raan_rad());
    const double thrust = d.session.body(0).vehicle().rcs_force_capability({1.0, 0.0, 0.0}).force_n;
    const double entry_mass = d.session.body(0).vehicle().mass_properties().mass_kg - 1500.0 - 500.0;
    const auto finite = d.plan(d.parameters(thrust), entry_mass);
    INFO(finite.reason);
    REQUIRE(finite.feasible);
    DragonDeorbit d_impulse(dragon_raan_rad());
    const auto impulsive = d_impulse.plan(d_impulse.parameters(0.0), entry_mass);
    REQUIRE(impulsive.feasible);
    double finite_dt = 0.0, impulse_dt = 0.0, finite_burn = 0.0, impulse_burn = 0.0;
    const double gamma_finite = d.fly_to_interface(finite, finite_dt, finite_burn);
    const double gamma_impulse = d_impulse.fly_to_interface(impulsive, impulse_dt, impulse_burn);
    std::printf("[validation] Draco deorbit %.1f m/s (%.0f kg planned), burn %.0f s: finite plan -> gamma %.3f deg "
                "(planned %.3f), interface %+.1f s; impulsive plan -> gamma %.3f deg, %+.1f s\n",
                finite.delta_v_mps, finite.propellant_kg, finite_burn, gamma_finite / deg,
                finite.entry_flight_path_rad / deg, finite_dt, gamma_impulse / deg, impulse_dt);
    CHECK(std::abs(gamma_finite - finite.entry_flight_path_rad) < 0.03 * deg);
    CHECK(std::abs(finite_dt) < 5.0);
    CHECK(std::abs(gamma_impulse - impulsive.entry_flight_path_rad) > std::abs(gamma_finite - finite.entry_flight_path_rad));
}

TEST_CASE("Deorbit plan predicts the entry with the entry mass", "[dragon][deorbit]") {
    REQUIRE(dragon_raan_rad() >= 0.0);
    DragonDeorbit d(dragon_raan_rad());
    const double stack = d.session.body(0).vehicle().mass_properties().mass_kg;
    auto p = d.parameters(0.0);
    const auto with_stack = d.plan(p, 0.0);  // default: the stack's mass after the burn
    p.entry_mass_kg = stack - 1500.0 - 500.0;
    const auto with_capsule = d.plan(p, 0.0);
    REQUIRE(with_stack.feasible);
    REQUIRE(with_capsule.feasible);
    // the capsule's plan, checked independently: coast from the impulse to the interface, predict with the entry mass
    const auto& s = d.session.body(0).state();
    auto at_node = orbit::propagate_coast(d.session.environment().gravity, {s.position_gcrf_m, s.velocity_gcrf_mps},
                                          d.session.epoch(), with_capsule.node.epoch);
    at_node.state.velocity_mps += with_capsule.node.delta_v_gcrf_mps;
    const integrators::Event<6> interface{
        "entry interface",
        [&](double t_s, const integrators::State<6>& y) {
            const Vec3 r = transpose(spice_earth().fixed_to_gcrf(with_capsule.node.epoch + t_s).rotation) * Vec3{y[0], y[1], y[2]};
            return frames::fixed_to_geodetic(r).height_m - 120000.0;
        },
        integrators::Crossing::Falling};
    const auto ei = orbit::propagate_coast(d.session.environment().gravity, at_node.state, with_capsule.node.epoch,
                                           with_capsule.node.epoch + 6000.0, std::span(&interface, 1));
    REQUIRE(ei.event_index == 0);
    const auto tr = spice_earth().fixed_to_gcrf(ei.epoch);
    double downrange[2];
    const double masses[2] = {p.entry_mass_kg, stack};
    for (int k = 0; k < 2; ++k) {
        double sum = 0.0;
        for (const double sign : {1.0, -1.0}) {
            gnc::EntryPredictorState start;
            start.position_itrf_m = transpose(tr.rotation) * ei.state.position_m;
            start.velocity_itrf_mps =
                transpose(tr.rotation) * (ei.state.velocity_mps - cross(vehicle::earth_rotation_vector(tr), ei.state.position_m));
            start.bank_rad = sign * p.nominal_bank_rad;
            sum += 0.5 * gnc::predict_entry(d.capsule_model(masses[k]), start, start.bank_rad, d.session.capsule_entry_target(),
                                            p.gate_height_m, 0.0, 2.0)
                             .downrange_error_m;
        }
        downrange[k] = sum;
    }
    std::printf("[validation] deorbit plans: stack mass -> burn %+.0f s / %.2f m/s, capsule mass -> %+.0f s / %.2f m/s; "
                "independent prediction of the capsule plan %.2f km (stack mass %.2f km)\n",
                with_stack.node.epoch - d.t0, with_stack.delta_v_mps, with_capsule.node.epoch - d.t0,
                with_capsule.delta_v_mps, downrange[0] / 1000.0, downrange[1] / 1000.0);
    CHECK(std::abs(downrange[0]) < 1000.0);
    CHECK(std::abs(downrange[1]) > std::abs(downrange[0]));
}

namespace {

struct ReturnResult {
    bool reached_gate = false;
    double burn_s = 0.0;
    double miss_m = 0.0;
    double speed_mps = 0.0;
    double peak_g = 0.0;
    double entry_interface_s = -1.0;  // seconds after the burn's end
    bool cone_closed_at_interface = false;
    bool trunk_separated_at_interface = false;
    double rcs_left_kg = 0.0;
    double entry_rcs_kg = 0.0;  // Draco propellant from the interface to the gate
};

// Flies a planned capsule return to the gate (or 48 h).
ReturnResult fly_capsule_return(sim::FlightSession& session, time::Epoch t0) {
    ReturnResult r;
    double ignition = -1.0, cutoff = -1.0, rcs_at_interface = 0.0;
    // the drogues follow the gate within one step (d4)
    while (static_cast<int>(session.capsule_return_step()) < static_cast<int>(sim::CapsuleReturnStep::Gate) &&
           session.epoch() - t0 < 2.0 * 86400.0) {
        const auto& body = session.body(session.controlled_index());
        if (body.impacted()) break;
        double step = 1.0;
        if (session.autopilot().state == sim::AutopilotState::Burn) {
            const double to_ignition = session.autopilot().burn.time_to_ignition_s;
            if (to_ignition > 900.0) step = to_ignition - 600.0;
            if (ignition < 0.0 && session.autopilot().burn.ignite) ignition = session.epoch() - t0;
        } else if (cutoff < 0.0 && ignition >= 0.0) {
            cutoff = session.epoch() - t0;
        }
        session.advance_to(session.epoch() + step, 1000000);
        const auto tel = session.telemetry(session.controlled_index());
        if (r.entry_interface_s < 0.0 && cutoff >= 0.0 && tel.altitude_m < 120000.0) {
            r.entry_interface_s = session.epoch() - t0 - cutoff;
            r.cone_closed_at_interface = !session.body(session.controlled_index()).vehicle().nose_cone_open();
            r.trunk_separated_at_interface = session.body_count() == 2;
            rcs_at_interface = session.body(session.controlled_index()).vehicle().rcs_propellant_kg();
        }
        r.peak_g = std::max(r.peak_g, norm(session.body(session.controlled_index()).aero_acceleration_body_mps2()) /
                                          constants::standard_gravity_mps2);
        if (std::getenv("STARSIM_VERBOSE") && tel.altitude_m < 130000.0 && std::fmod(session.epoch() - t0, 20.0) < 1.0) {
            std::printf("t %.0f %s h %.0f v %.0f\n", session.epoch() - t0, sim::to_string(session.capsule_return_step()),
                        tel.altitude_m, tel.speed_relative_mps);
        }
    }
    r.burn_s = cutoff - ignition;
    const size_t i = session.controlled_index();
    r.reached_gate = static_cast<int>(session.capsule_return_step()) >= static_cast<int>(sim::CapsuleReturnStep::Gate);
    if (session.entry_guidance(i)) {
        const auto tr = spice_earth().fixed_to_gcrf(session.epoch());
        const Vec3 pos = transpose(tr.rotation) * session.body(i).state().position_gcrf_m;
        r.miss_m = horizontal_miss_m(pos, session.entry_guidance(i)->target().position_itrf_m);
    }
    r.speed_mps = session.telemetry(i).speed_relative_mps;
    r.rcs_left_kg = session.body(i).vehicle().rcs_propellant_kg();
    r.entry_rcs_kg = rcs_at_interface - r.rcs_left_kg;
    return r;
}

}  // namespace

TEST_CASE("Crew Dragon returns from orbit to the drogue gate", "[dragon][mission]") {
    REQUIRE(dragon_raan_rad() >= 0.0);
    DragonDeorbit d(dragon_raan_rad());
    auto& s = d.session;
    const double rcs0 = s.body(0).vehicle().rcs_propellant_kg();
    REQUIRE(s.plan_capsule_return());
    const auto& plan = *s.ship_return_plan();
    const auto r = fly_capsule_return(s, d.t0);
    std::printf("[validation] Crew Dragon return: deorbit %.1f m/s (%.0f kg) after %.1f h, burn %.0f s (published 7.5-12 "
                "min), interface %.0f s after the burn; gate miss %.2f km at %.0f m/s (%.0f km/h, published ~560), peak "
                "%.2f g; Dracos %.0f kg in all, %.1f kg in the entry, %.0f kg left\n",
                plan.delta_v_mps, plan.propellant_kg, (plan.node.epoch - d.t0) / 3600.0, r.burn_s, r.entry_interface_s,
                r.miss_m / 1000.0, r.speed_mps, r.speed_mps * 3.6, r.peak_g, rcs0 - r.rcs_left_kg, r.entry_rcs_kg,
                r.rcs_left_kg);
    CHECK(r.reached_gate);
    CHECK(r.trunk_separated_at_interface);
    CHECK(r.cone_closed_at_interface);
    CHECK(r.miss_m < 2000.0);
    CHECK(r.speed_mps == Catch::Approx(156.0).epsilon(0.15));
    CHECK(r.rcs_left_kg > sim::FlightSession::capsule_return_reserve_kg);
    const auto& capsule = s.body(s.controlled_index());
    CHECK_FALSE(capsule.impacted());
    CHECK_FALSE(capsule.crew_load_warning());
    CHECK(capsule.vehicle().stages().size() == 1);
    CHECK(capsule.vehicle().stages().front().config.id == "capsule");
    // After the gate the capsule comes down under its parachutes (d4) and is gewassert; then the session coasts again
    s.advance_to(s.epoch() + 600.0, 1000000);
    CHECK(s.landing_outcome(s.controlled_index()) == sim::LandingOutcome::SplashedDown);
    const auto before = s.epoch();
    const auto reached = s.advance_to(before + 3600.0, 2000);
    CHECK(reached - before > 3000.0);
}

TEST_CASE("No pass within reach refuses the capsule return", "[dragon][mission]") {
    DragonDeorbit d(0.0, 10.0 * deg);  // the zone at 32.6 N is never overflown
    const auto mode = gnc::inertial_hold(d.session.body(0).state().attitude);
    d.session.set_attitude_mode(mode);
    CHECK_FALSE(d.session.plan_capsule_return());
    REQUIRE(d.session.ship_return_plan() != nullptr);
    CHECK_FALSE(d.session.ship_return_plan()->reason.empty());
    CHECK(d.session.attitude_mode().kind == gnc::AttitudeModeKind::Inertial);
    CHECK(d.session.capsule_return_step() == sim::CapsuleReturnStep::None);
    CHECK(d.session.autopilot().state == sim::AutopilotState::Off);
}

TEST_CASE("The attitude mode ends with the autopilot and with a burn", "[dragon][attitude]") {
    DragonDeorbit d(0.0);
    auto& s = d.session;
    s.set_attitude_mode(gnc::inertial_hold(s.body(0).state().attitude));
    s.disengage_autopilot();
    CHECK(s.attitude_mode().kind == gnc::AttitudeModeKind::Free);
    s.set_attitude_mode(gnc::inertial_hold(s.body(0).state().attitude));
    const auto& st = s.body(0).state();
    REQUIRE(s.execute_burn({s.epoch() + 300.0, math::normalized(st.velocity_gcrf_mps) * 0.5}));
    CHECK(s.attitude_mode().kind == gnc::AttitudeModeKind::Free);
}

TEST_CASE("place_in_orbit forgets an earlier payload release", "[dragon][sim]") {
    DragonDeorbit d(0.0);
    auto& s = d.session;
    s.set_payload_separation(sim::PayloadSeparation{"capsule", 30.0, 0.3, 600.0});
    REQUIRE(s.separate_payload());  // the capsule leaves its trunk as a "payload"; the cone timer runs
    const orbit::KeplerianElements circular{constants::wgs84_semi_major_axis_m + 420e3, 0.0, 51.6 * deg, 0.0, 0.0, 0.0};
    s.place_in_orbit("crew_dragon", orbit::state_from_elements(circular, s.gm_earth_m3ps2()), s.epoch());
    s.advance_to(s.epoch() + 700.0, 1000000);
    CHECK_FALSE(s.body(0).vehicle().nose_cone_open());
    CHECK_FALSE(s.separate_payload());
}

namespace {

struct CampaignCase {
    double gamma_rad = 0.0, mass_factor = 1.0, aero_factor = 1.0;
};

// Seed 1-12, the spec's dispersion: entry angle +-0.1 deg, mass +-3 % (RCS tank), truth aero +-10 %, from the nominal
// (planned) entry interface state.
CampaignCase campaign_case(int seed) {
    std::mt19937 rng(static_cast<unsigned>(seed));
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    CampaignCase c;
    c.gamma_rad = (-2.0 + 0.1 * u(rng)) * deg;
    c.mass_factor = 1.0 + 0.03 * u(rng);
    c.aero_factor = 1.0 + 0.10 * u(rng);
    return c;
}

GateResult fly_campaign_case(int seed, double& peak_g, bool& crew_warning) {
    test::spice_session();
    const auto t0 = spice::SpiceTimeConverter{}.from_utc({2026, 10, 14, 18, 0, 0.0});
    const auto zone = config::load_splashdown_zone(STARSIM_CONFIG_DIR, "pacific_san_diego");
    const auto c = campaign_case(seed);
    const double nominal_mass = capsule_config().stages.front().dry_mass_kg + 2100.0;
    const auto cfg = capsule_config(2100.0 + (c.mass_factor - 1.0) * nominal_mass);
    const vehicle::Vehicle nominal(capsule_config(), STARSIM_CONFIG_DIR);
    const vehicle::Vehicle capsule(cfg, STARSIM_CONFIG_DIR);
    const double azimuth = 120.0 * deg;
    const double speed = 7500.0;
    // the nominal entry (as planned), then dispersed
    static double offset = 0.0;
    static const double range = nominal_entry_range_m(zone, offset, azimuth, speed, -2.0 * deg, 30.0 * deg, t0, nominal);
    sim::FlightSession session(spice_ephemeris(), spice_earth(), STARSIM_CONFIG_DIR);
    session.set_splashdown_zone(zone);
    session.place_entering_capsule(
        cfg, entry_interface_state(zone, range, azimuth, speed, c.gamma_rad, 30.0 * deg, t0, capsule, offset), t0);
    session.set_aero_dispersion(0, c.aero_factor);
    const auto g = fly_capsule_to_gate(session, t0);
    peak_g = g.peak_g;
    crew_warning = session.body(0).crew_load_warning();
    std::printf("[validation] capsule campaign %2d: gamma %.3f deg, mass %+.1f %%, aero %+.1f %% -> %s, miss "
                "%.2f km, %.0f m/s, peak %.2f g%s%s\n",
                seed, c.gamma_rad / deg, 100.0 * (c.mass_factor - 1.0), 100.0 * (c.aero_factor - 1.0), g.reached ? "gate" : "NO GATE", g.miss_m / 1000.0, g.speed_mps, g.peak_g,
                crew_warning ? ", crew warning" : "", session.body(0).impacted() ? ", BROKEN UP" : "");
    CHECK_FALSE(session.body(0).impacted());
    return g;
}

}  // namespace

TEST_CASE("Capsule entry campaign reaches the drogue gate", "[dragon][entry][validation][slow]") {
    int warnings = 0;
    double worst_miss = 0.0;
    for (int seed = 1; seed <= 12; ++seed) {
        double peak_g = 0.0;
        bool warning = false;
        const auto g = fly_campaign_case(seed, peak_g, warning);
        CHECK(g.reached);
        CHECK(g.miss_m < 2000.0);
        // the gate speed is checked against the published value in the nominal tests: a truth aero dispersed by -10 %
        // falls ~5 % faster than the real capsule would
        CHECK(g.speed_mps == Catch::Approx(156.0).epsilon(0.20));
        warnings += warning ? 1 : 0;
        worst_miss = std::max(worst_miss, g.miss_m);
    }
    std::printf("[validation] capsule campaign: worst miss %.2f km, crew warnings %d of 12\n", worst_miss / 1000.0, warnings);
}

TEST_CASE("Capsule entry single case", "[.][dragon]") {
    const char* seed = std::getenv("STARSIM_CAPSULE_SEED");
    double peak_g = 0.0;
    bool warning = false;
    const auto g = fly_campaign_case(seed ? std::atoi(seed) : 1, peak_g, warning);
    CHECK(g.miss_m < 2000.0);
}

TEST_CASE("A finite deorbit burn ignites after the planning lead", "[dragon][deorbit]") {
    // Review (d3) I1: the lead limited the burn's centre; a ~17 min Draco burn ignites half its length earlier.
    REQUIRE(dragon_raan_rad() >= 0.0);
    DragonDeorbit d(dragon_raan_rad());
    const auto& v = d.session.body(0).vehicle();
    const auto rcs = v.rcs_force_capability({1.0, 0.0, 0.0});
    const double mass = v.mass_properties().mass_kg;
    auto p = d.parameters(rcs.force_n);
    p.entry_mass_kg = mass - 2100.0;
    const auto first = d.plan(p, 0.0);
    REQUIRE(first.feasible);
    // the earliest burn just before the first plan's: the solution sits at the lead
    p.lead_s = (first.node.epoch - d.session.epoch()) - 100.0;
    const auto tight = d.plan(p, 0.0);
    if (!tight.feasible) {  // that pass is out of reach now (no later one in the window): refused, not ignited early
        CHECK_FALSE(tight.reason.empty());
        return;
    }
    const double duration = gnc::burn_duration_s(tight.delta_v_mps, mass, rcs.force_n, rcs.mass_flow_kgps);
    const double ignition_after_planning = (tight.node.epoch - 0.5 * duration) - d.session.epoch();
    std::printf("[validation] deorbit with the lead at the solution: ignition %.0f s after planning (lead %.0f s)\n",
                ignition_after_planning, p.lead_s);
    CHECK(ignition_after_planning >= p.lead_s - 1.0);
}

TEST_CASE("A capsule return in progress refuses a second plan", "[dragon][mission]") {
    // Review (d3) I2: a second call reset the return mid-way (the ship fallback then started the wrong entry guidance).
    REQUIRE(dragon_raan_rad() >= 0.0);
    DragonDeorbit d(dragon_raan_rad());
    REQUIRE(d.session.plan_capsule_return());
    const auto node_epoch = d.session.ship_return_plan()->node.epoch;
    CHECK_FALSE(d.session.plan_capsule_return());
    CHECK(d.session.capsule_return_step() == sim::CapsuleReturnStep::DeorbitBurn);
    CHECK(d.session.autopilot().state == sim::AutopilotState::Burn);
    REQUIRE(d.session.ship_return_plan() != nullptr);
    CHECK(d.session.ship_return_plan()->node.epoch == node_epoch);  // the plan in progress is kept
}

TEST_CASE("A new flight or a pilot burn ends a capsule return", "[dragon][mission]") {
    // Review (d3) I3: the return step survived a new flight; an RCS burn there then jettisoned the new vehicle's stage.
    REQUIRE(dragon_raan_rad() >= 0.0);
    DragonDeorbit d(dragon_raan_rad());
    REQUIRE(d.session.plan_capsule_return());
    const auto& st = d.session.body(0).state();
    REQUIRE(d.session.execute_burn({d.session.epoch() + 300.0, math::normalized(st.velocity_gcrf_mps) * 0.5}));
    CHECK(d.session.capsule_return_step() == sim::CapsuleReturnStep::None);
    REQUIRE(d.session.plan_capsule_return());
    d.session.place_body(capsule_config(), d.session.body(0).state(), d.session.epoch());
    CHECK(d.session.capsule_return_step() == sim::CapsuleReturnStep::None);
}

TEST_CASE("A capsule still on the second stage refuses the capsule return", "[dragon][mission]") {
    DragonDeorbit d(dragon_raan_rad());
    auto stack = config::stack_payload(config::load_vehicle(STARSIM_CONFIG_DIR, "falcon9_block5"),
                                       config::load_vehicle(STARSIM_CONFIG_DIR, "crew_dragon"));
    stack.stages.erase(stack.stages.begin());  // the second stage with Dragon on top, in orbit
    const auto s = d.session.body(0).state();
    d.session.place_body(stack, s, d.session.epoch());
    d.session.set_splashdown_zone(d.zone);
    CHECK_FALSE(d.session.plan_capsule_return());
    REQUIRE(d.session.ship_return_plan() != nullptr);
    CHECK(d.session.ship_return_plan()->reason == "Keine Rückkehr möglich: Kapsel noch auf der Oberstufe");
}

TEST_CASE("A capsule released from the second stage plans its return", "[dragon][mission]") {
    // Review d5 I1: the stages of a released payload kept their payload flag, so a capsule flown up on Falcon 9 was
    // refused as "noch auf der Oberstufe" and never reported as a capsule body.
    DragonDeorbit d(dragon_raan_rad());
    auto stack = config::stack_payload(config::load_vehicle(STARSIM_CONFIG_DIR, "falcon9_block5"),
                                       config::load_vehicle(STARSIM_CONFIG_DIR, "crew_dragon"));
    stack.stages.erase(stack.stages.begin(), stack.stages.begin() + 2);  // trunk + capsule, released
    REQUIRE(stack.stages.front().payload);
    const auto s = d.session.body(0).state();
    d.session.place_body(stack, s, d.session.epoch());
    d.session.set_splashdown_zone(d.zone);
    CHECK_FALSE(d.session.body(0).vehicle().payload_stacked());
    CHECK(d.session.plan_capsule_return());
}

TEST_CASE("Splitting a body keeps its truth aero dispersion on both parts", "[vehicle][dragon]") {
    vehicle::Vehicle stack(config::load_vehicle(STARSIM_CONFIG_DIR, "crew_dragon"), STARSIM_CONFIG_DIR);
    stack.set_hull_aero_scale(1.1);
    auto capsule = stack.separate(1);
    CHECK(capsule->hull_aero_scale() == 1.1);
    CHECK(stack.hull_aero_scale() == 1.1);
}
