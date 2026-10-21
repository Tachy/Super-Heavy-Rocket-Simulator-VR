// Refuelling r3: Starship rendezvous and probe/drogue docking at the depot.

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "proximity_test_support.hpp"
#include "refuelling_test_support.hpp"
#include "starsim/gnc/proximity_guidance.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/contact/probe_drogue_interface.hpp"
#include "starsim/contact/probe_drogue_mechanism.hpp"
#include "starsim/contact/station_collision.hpp"
#include "starsim/frames/lvlh.hpp"
#include "starsim/sim/station_limits.hpp"
#include "starsim/vehicle/port_mating.hpp"
#include "starsim/vehicle/vehicle.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {

const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;

std::unique_ptr<vehicle::Vehicle> ship_alone(const std::string& vehicle_id) {
    return std::make_unique<vehicle::Vehicle>(config::top_stage_config(config::load_vehicle(config_dir, vehicle_id)),
                                              config_dir);
}

}  // namespace

TEST_CASE("Tanker and HLS translate on all six axes; the Block 3 ship still has no -x", "[refuelling][rcs]") {
    for (const std::string id : {"starship_tanker", "starship_hls"}) {
        auto ship = ship_alone(id);
        ship->set_tank_mass(0, "lox_main", 7826.0);  // 10 t main propellant at O/F 3.6 (r1 translation case)
        ship->set_tank_mass(0, "ch4_main", 2174.0);
        INFO(id);
        for (const Vec3 u : {Vec3{1, 0, 0}, Vec3{-1, 0, 0}, Vec3{0, 1, 0}, Vec3{0, -1, 0}, Vec3{0, 0, 1}, Vec3{0, 0, -1}}) {
            CHECK(ship->rcs_pure_force_capability(u).force_n > 1000.0);
        }
        CHECK(ship->rcs_pure_force_capability({-1, 0, 0}).force_n ==
              Catch::Approx(ship->rcs_pure_force_capability({1, 0, 0}).force_n).epsilon(0.01));
    }
    auto block3 = ship_alone("starship_block3");
    CHECK(block3->rcs_pure_force_capability({-1, 0, 0}).force_n == 0.0);
}

TEST_CASE("RCS supply: main-tank cold gas counts the main tanks, Dragon its RCS tank", "[refuelling][rcs]") {
    auto tanker = ship_alone("starship_tanker");
    tanker->set_tank_mass(0, "lox_main", 20000.0);
    tanker->set_tank_mass(0, "ch4_main", 6000.0);
    CHECK(tanker->rcs_supply_kg() == Catch::Approx(26000.0));  // the headers are not RCS supply (landing reserve)
    CHECK(tanker->rcs_propellant_kg() == 0.0);
    const vehicle::Vehicle dragon(config::load_vehicle(config_dir, "crew_dragon"), config_dir);
    CHECK(dragon.rcs_supply_kg() == Catch::Approx(dragon.rcs_propellant_kg()));
    CHECK(dragon.rcs_supply_kg() > 0.0);
}

namespace {

// A config dir with Block 3, the engines and propellants and one test vehicle (as test_refuelling_vehicles.cpp; own
// temp prefix - ctest runs cases in parallel).
std::filesystem::path config_with(const std::string& name, const std::string& vehicle_toml) {
    const auto dir = std::filesystem::temp_directory_path() / ("starsim_r3_" + name);
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir / "vehicles");
    std::filesystem::copy(config_dir / "engines", dir / "engines", std::filesystem::copy_options::recursive);
    std::filesystem::copy_file(config_dir / "propellants.toml", dir / "propellants.toml");
    std::filesystem::copy_file(config_dir / "vehicles" / "starship_block3.toml", dir / "vehicles" / "starship_block3.toml");
    std::ofstream(dir / "vehicles" / "test_vehicle.toml") << vehicle_toml;
    return dir;
}

std::string reused_stage(const std::string& from, const std::string& additions) {
    return "[vehicle]\nid = \"test_vehicle\"\nname = \"Test\"\npropellant = \"methalox\"\n"
           "stages_bottom_to_top = [\"booster\"]\n\n[[stage]]\nfrom = \"" + from + "\"\n" + additions;
}

const std::string docking_pod = "[[stage.rcs_pod]]\nid = \"dock\"\nstation_m = 46.0\nradius_m = 4.5\n"
                                "azimuths_deg = [0.0, 180.0]\nthrust_n = 1500.0\ndirections = [\"aft\"]\n";

size_t aft_thrusters(const config::StageConfig& s) {
    return static_cast<size_t>(std::count_if(s.rcs.thrusters.begin(), s.rcs.thrusters.end(),
                                             [](const config::RcsThrusterConfig& t) { return t.direction.x < -0.99; }));
}

}  // namespace

TEST_CASE("A reused stage may add RCS pods, not other RCS keys", "[refuelling][config]") {
    const auto tanker = config::load_vehicle(config_dir, "starship_tanker");
    CHECK(aft_thrusters(tanker.stages.back()) == 2);
    const auto block3 = config::load_vehicle(config_dir, "starship_block3");

    // the booster has [stage.rcs] (its forward pod): pods may be added
    auto dir = config_with("rcs_pod_ok", reused_stage("starship_block3:booster", docking_pod));
    const auto added = config::load_vehicle(dir, "test_vehicle");
    CHECK(added.stages.back().rcs.thrusters.size() == block3.stages.front().rcs.thrusters.size() + 2);
    CHECK(aft_thrusters(added.stages.back()) == 2);
    std::filesystem::remove_all(dir);

    dir = config_with("rcs_table", reused_stage("starship_block3:booster", "[stage.rcs]\nisp_s = 70.0\n"));
    CHECK_THROWS_WITH(config::load_vehicle(dir, "test_vehicle"),
                      Catch::Matchers::ContainsSubstring("cannot be set on a reused stage"));
    std::filesystem::remove_all(dir);
}

TEST_CASE("An rcs_pod on a reused stage without RCS is an error", "[refuelling][config]") {
    const auto dir = config_with("rcs_none", reused_stage("bare:booster", docking_pod));
    std::ofstream(dir / "vehicles" / "bare.toml")
        << "[vehicle]\nid = \"bare\"\nname = \"Bare\"\npropellant = \"methalox\"\nstages_bottom_to_top = [\"booster\"]\n\n"
           "[[stage]]\nid = \"booster\"\nname = \"Bare\"\nlength_m = 10.0\ndiameter_m = 9.0\ndry_mass_kg = 1000.0\n"
           "dry_cg_station_m = 5.0\n";
    CHECK(config::load_vehicle(dir, "bare").stages.front().rcs.thrusters.empty());
    CHECK_THROWS_WITH(config::load_vehicle(dir, "test_vehicle"), Catch::Matchers::ContainsSubstring("rcs_pod needs"));
    std::filesystem::remove_all(dir);
}

TEST_CASE("The depot is a target: LVLH attitude nose forward, dorsal to nadir, its approach limits",
          "[refuelling][station][config]") {
    const auto depot = config::load_vehicle(config_dir, "starship_depot");
    const auto& s = depot.stages.back();
    REQUIRE(s.body_in_lvlh_rad);
    CHECK(norm(*s.body_in_lvlh_rad) == 0.0);
    REQUIRE(s.station_proximity.keep_out_radius_m);
    CHECK(*s.station_proximity.keep_out_radius_m == 200.0);
    CHECK(s.station_proximity.approach_ellipsoid_m->x == 2000.0);
    CHECK(*s.station_proximity.waypoint0_below_m == 400.0);
    const auto iss = config::load_vehicle(config_dir, "iss");
    CHECK_FALSE(iss.stages.front().station_proximity.keep_out_radius_m);  // the ISS keeps the code defaults
}

TEST_CASE("Station limits replace only the keys the target sets", "[refuelling][station]") {
    config::StationProximityConfig s;
    s.keep_out_radius_m = 150.0;
    s.waypoint1_range_m = 180.0;
    s.waypoint0_below_m = 350.0;
    s.approach_ellipsoid_m = Vec3{1500.0, 800.0, 800.0};
    const gnc::ProximityParameters defaults;
    const auto p = sim::with_station_limits(gnc::ProximityParameters{}, s);
    CHECK(p.keep_out_radius_m == 150.0);
    CHECK(p.waypoint1_range_m == 180.0);
    CHECK(p.waypoint0_below_m == 350.0);
    CHECK(p.waypoint2_range_m == defaults.waypoint2_range_m);
    CHECK(p.flyaround_radius_m == defaults.flyaround_radius_m);
    CHECK(sim::with_station_limits(gnc::DepartureParameters{}, s).ellipsoid_semi_axes_m.x == 1500.0);
    CHECK(sim::with_station_limits(mission::PhasingParameters{}, s).waypoint0_below_m == 350.0);
    CHECK(sim::with_station_limits(gnc::ProximityParameters{}, config::StationProximityConfig{}).keep_out_radius_m ==
          defaults.keep_out_radius_m);
}

TEST_CASE("add_station takes the depot alone; its drogues face nadir, its nose the velocity", "[refuelling][station]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    CHECK_THROWS_AS(f.session.add_station(config::load_vehicle(config_dir, "starship_depot"), f.depot, f.t0),
                    std::logic_error);  // the stack: a station is one stage
    auto no_target = refuelling_test::tanker_config(100.0e3);
    CHECK_THROWS_AS(f.session.add_station(no_target, f.depot, f.t0), std::logic_error);  // no [stage.station]
    f.session.add_station(refuelling_test::depot_config(f.depot_propellant_kg), f.depot, f.t0);
    CHECK_THROWS_AS(f.session.add_station(refuelling_test::depot_config(1.0), f.depot, f.t0), std::logic_error);
    f.session.advance_to(f.t0 + 1.0);
    const auto& st = f.session.body(0).state();
    const math::Mat3 to_lvlh =
        transpose(frames::lvlh_to_inertial({st.position_gcrf_m, st.velocity_gcrf_mps}).to_matrix());
    const auto port = vehicle::port_kinematics(f.session.body(0), "transfer_dock");
    CHECK((to_lvlh * column(port.axes_gcrf, 0)).z == Catch::Approx(1.0).margin(1e-6));  // drogues to nadir
    CHECK((to_lvlh * column(port.axes_gcrf, 2)).x == Catch::Approx(1.0).margin(1e-6));  // nose along +V-bar
}

namespace {

// CW model of the tanker under the depot: target port axes in LVLH x = nadir (dorsal normal), z = +V (nose),
// y = z x x = -y_lvlh. Authority along the target port axes (Task 1 tanker at ~490 t): +-x_t = the ship's +-z,
// +-y_t its +-y, +-z_t its +-x.
proximity_test::CwSim depot_cw() {
    proximity_test::CwSim s;
    s.n = std::sqrt(constants::earth_gm_m3ps2 / std::pow(constants::wgs84_semi_major_axis_m + 200.0e3, 3));
    s.port_to_lvlh = math::Mat3::from_columns({0, 0, 1}, {0, -1, 0}, {1, 0, 0});
    s.port_lever_lvlh = {2.0, 0.0, 5.0};  // [test] depot CG ~18 m up the hull, port at 20 m, 5 m off the axis
    s.chaser_lever_m = 5.0;
    const double m = 490.0e3;
    s.authority = gnc::AxisAcceleration{{11338.0 / m, 11338.0 / m, 5670.0 / m, 5670.0 / m, 3000.0 / m, 3000.0 / m}};
    s.propellant_kg = 200.0e3;
    s.kg_per_mps = m / (70.0 * constants::standard_gravity_mps2);  // cold gas Isp 70 s
    return s;
}

gnc::ProximityParameters depot_parameters() {
    gnc::ProximityParameters p;
    p.contact_plane_m = 0.80;  // probe/drogue first touch (Task 5: cone depth + stroke)
    p.envelope = gnc::probe_drogue_capture_envelope();
    return p;
}

bool saw(const gnc::ProximityGuidance& g, gnc::ProximityPhase phase) {
    for (const auto& e : g.events()) {
        if (e.phase == phase) return true;
    }
    return false;
}

}  // namespace

TEST_CASE("A nadir port is approached along the R-bar from below, no fly-around", "[refuelling][proximity]") {
    CHECK(gnc::docking_approach({0.0, 0.0, 1.0}) == gnc::DockingApproach::RBarBelow);
    CHECK(gnc::docking_approach({1.0, 0.0, 0.0}) == gnc::DockingApproach::VBar);  // ISS paths unchanged
    CHECK(gnc::docking_approach({0.0, 0.0, -1.0}) == gnc::DockingApproach::RBar);
    const auto path = gnc::proximity_path(gnc::DockingApproach::RBarBelow, gnc::ProximityParameters{});
    REQUIRE(path.size() == 3);
    CHECK(path[1].on_axis);
    CHECK(path[2].on_axis);
}

TEST_CASE("Tanker from Waypoint 0 to contact under the depot inside the probe/drogue envelope",
          "[refuelling][proximity]") {
    auto s = depot_cw();
    s.cg = {{0.0, 0.0, 400.0}, {}};  // Waypoint 0 at rest, 400 m below the depot CG
    gnc::ProximityGuidance g(gnc::DockingApproach::RBarBelow, depot_parameters());
    const double t = proximity_test::run(g, s, 3.0 * 3600.0);
    REQUIRE(g.contact());
    INFO(g.contact()->envelope.reason);
    CHECK(g.contact()->envelope.inside);
    CHECK(std::hypot(g.contact()->port.position_m.y, g.contact()->port.position_m.z) < 0.10);
    CHECK_FALSE(saw(g, gnc::ProximityPhase::Transfer));  // straight up the axis
    CHECK_FALSE(saw(g, gnc::ProximityPhase::Flyaround));
    CHECK_FALSE(saw(g, gnc::ProximityPhase::Abort));
    CHECK(saw(g, gnc::ProximityPhase::Approach));
    CHECK(t < 2.0 * 3600.0);
}

TEST_CASE("A retreat below the depot stops at Waypoint 2 - the -x thrusters hold the Coriolis drift",
          "[refuelling][proximity]") {
    auto s = depot_cw();
    s.cg = {{0.0, 0.0, 400.0}, {}};
    gnc::ProximityGuidance g(gnc::DockingApproach::RBarBelow, depot_parameters());
    proximity_test::run(g, s, 3.0 * 3600.0);
    REQUIRE(g.phase() == gnc::ProximityPhase::Contact);
    g.capture_failed(s.input(), "Überlast");
    for (double t = 0.0; t < 1800.0 && g.phase() != gnc::ProximityPhase::Waypoint; t += 0.05) {
        s.step(g.update(s.input()).translation_fraction_port, 0.05);
    }
    REQUIRE(g.phase() == gnc::ProximityPhase::Waypoint);  // Waypoint 2, waiting for the go
    for (double t = 0.0; t < 600.0; t += 0.05) s.step(g.update(s.input()).translation_fraction_port, 0.05);
    const auto in = s.input();
    CHECK(in.port.position_m.x == Catch::Approx(20.0).margin(1.0));
    CHECK(std::hypot(in.port.position_m.y, in.port.position_m.z) < 1.0);  // held, no forward drift
    CHECK(norm(in.port.velocity_mps) < 0.01);
}

TEST_CASE("The probe/drogue envelope is the spec's", "[refuelling][proximity]") {
    const auto e = gnc::probe_drogue_capture_envelope();
    CHECK(e.axial_min_mps == 0.03);
    CHECK(e.axial_max_mps == 0.10);
    CHECK(e.offset_max_m == 0.10);
    CHECK(e.angle_max_rad == Catch::Approx(1.0 * constants::deg_to_rad));
    CHECK(gnc::ProximityParameters{}.envelope.axial_min_mps == 0.05);  // IDSS default unchanged
}

namespace {

contact::ProbeDrogueParameters depot_probes() {
    const auto tanker = config::load_vehicle(config_dir, "starship_tanker");
    const auto depot = config::load_vehicle(config_dir, "starship_depot");
    return contact::probe_drogue_parameters(tanker.stages.back().docking_ports.front(),
                                            depot.stages.back().docking_ports.front());
}

const math::Mat3 mated_axes = math::Mat3::from_columns({-1, 0, 0}, {0, -1, 0}, {0, 0, 1});

contact::DockingPose pose_at(const Vec3& position_m, const Vec3& velocity_mps = {},
                             const math::Mat3& axes = mated_axes) {
    return {position_m, velocity_mps, axes, {}};
}

}  // namespace

TEST_CASE("Probe/drogue geometry from the r1 ports: probes and drogues pair up when mated", "[refuelling][contact]") {
    const auto p = depot_probes();
    REQUIRE(p.probes_m.size() == 4);
    REQUIRE(p.drogues_m.size() == 4);
    std::vector<Vec3> mated_m, drogues_yz_m;
    for (const auto& probe : p.probes_m) {
        CHECK(probe.x == 0.0);  // in the interface plane
        mated_m.push_back({0.0, -probe.y, probe.z});
    }
    for (const auto& d : p.drogues_m) drogues_yz_m.push_back({0.0, d.y, d.z});
    CHECK(proximity_test::probe_pairing_problem(mated_m, drogues_yz_m, 1e-9) == std::nullopt);
}

TEST_CASE("Probe/drogue contact: free above the mouths, centring in the cone, seated heads latch",
          "[refuelling][contact]") {
    const auto p = depot_probes();
    const double first = p.cone_depth_m + p.extension_m;  // port separation at the first touch
    auto c = contact::probe_drogue_contact(pose_at({first + 0.01, 0.0, 0.0}), p.extension_m, p);
    CHECK_FALSE(c.any_contact);
    CHECK(c.gap_m == Catch::Approx(0.01).margin(1e-9));

    // 0.08 m off-centre (along y), halfway down the cone: the walls push towards the axis and out
    c = contact::probe_drogue_contact(pose_at({first - 0.40, 0.08, 0.0}), p.extension_m, p);
    REQUIRE(c.any_contact);
    CHECK(c.petal_contact);
    CHECK(c.force_port_n.y < 0.0);
    CHECK(c.force_port_n.x > 0.0);
    CHECK_FALSE(c.latch_ready);

    // centred, heads in the seats at full extension: latch ready, no penetration force
    c = contact::probe_drogue_contact(pose_at({p.extension_m + 0.005, 0.0, 0.0}), p.extension_m, p);
    CHECK(c.latch_ready);
    CHECK(norm(c.force_port_n) < 1e-6);

    // mated (retracted): every head on its seat, no force
    c = contact::probe_drogue_contact(pose_at({0.0, 0.0, 0.0}), 0.0, p);
    CHECK(norm(c.force_port_n) < 1e-6);
}

TEST_CASE("Probe/drogue contact: a head beside the mouth strikes the plate, no latch; tilted heads give a torque",
          "[refuelling][contact]") {
    const auto p = depot_probes();
    const double first = p.cone_depth_m + p.extension_m;
    auto c = contact::probe_drogue_contact(pose_at({first - 0.05, 0.6, 0.0}, {-0.05, 0.0, 0.0}), p.extension_m, p);
    CHECK(c.tip_strike);
    CHECK(c.force_port_n.x > 0.0);
    CHECK_FALSE(c.latch_ready);

    // rolled 180 deg about the docking axis and pushed in: no head reaches a seat it could latch in (the roll limit)
    const math::Mat3 rolled = mated_axes * math::Mat3::from_columns({1, 0, 0}, {0, -1, 0}, {0, 0, -1});
    c = contact::probe_drogue_contact(pose_at({p.extension_m, 0.0, 0.0}, {}, rolled), p.extension_m, p);
    CHECK_FALSE(c.latch_ready);

    // pitched 2 deg about y: the deeper pair touches first - the contact force has a moment about the head centre
    const double a = 2.0 * constants::deg_to_rad;
    const math::Mat3 pitch =
        math::Mat3::from_columns({std::cos(a), 0, -std::sin(a)}, {0, 1, 0}, {std::sin(a), 0, std::cos(a)});
    c = contact::probe_drogue_contact(pose_at({p.extension_m + 0.10, 0.0, 0.0}, {}, mated_axes * pitch),
                                      p.extension_m, p);
    REQUIRE(c.any_contact);
    CHECK(std::abs(c.torque_port_nm.y) > 0.0);
}

TEST_CASE("IDSS reach and contact plane are unchanged by the generalised mechanism", "[refuelling][docking]") {
    const contact::DockingMechanism idss;
    CHECK(idss.full_extension_m() == 0.30);
    CHECK(idss.first_contact_m() == Catch::Approx(0.70));
    CHECK(idss.reach_m() == Catch::Approx(0.70 + 1.2));
}

namespace {

contact::DockingMechanism depot_mechanism() {
    const auto tanker = config::load_vehicle(config_dir, "starship_tanker");
    const auto depot = config::load_vehicle(config_dir, "starship_depot");
    return contact::DockingMechanism(contact::probe_drogue_mechanism(tanker.stages.back().docking_ports.front(),
                                                                     depot.stages.back().docking_ports.front()));
}

// Relative 1-D dynamics of a 490 t tanker against a 1,600 t depot at 200 Hz, centred, from 2 cm before the first
// touch at `closing_mps`, until `done` or `limit_s`; returns the elapsed seconds.
template <class Done>
double push_in(contact::DockingMechanism& m, double closing_mps, double limit_s, Done done) {
    const double mc = 490.0e3, ms = 1600.0e3, mu = mc * ms / (mc + ms), dt = 0.005;
    double x = m.first_contact_m() + 0.02, v = -closing_mps;
    double t = 0.0;
    for (; t < limit_s && !done(); t += dt) {
        const auto w = m.update(t, {{x, 0.0, 0.0}, {v, 0.0, 0.0}, mated_axes, {}}, mc, 1.0e8, dt);
        v += w.force_port_n.x / mu * dt;
        x += v * dt;
    }
    return t;
}

}  // namespace

TEST_CASE("Probe/drogue mechanism: 0.10 m/s soft capture within the loads, retraction, latches, docked",
          "[refuelling][docking]") {
    auto m = depot_mechanism();
    CHECK(m.first_contact_m() == Catch::Approx(0.80));
    const double t = push_in(m, 0.10, 1500.0, [&] { return m.phase() == contact::DockingPhase::Docked; });
    INFO(m.status());
    CHECK(m.phase() == contact::DockingPhase::Docked);
    CHECK(m.peaks().compression_n < m.parameters().dynamic_compression_limit_n);
    CHECK(m.peaks().compression_n > 10.0e3);
    CHECK(t < 20.0 * 60.0);  // soft capture -> docked: attenuation 120 s + retraction 300 s + latches 180 s
}

TEST_CASE("Probe/drogue mechanism: 0.25 m/s overloads", "[refuelling][docking]") {
    auto m = depot_mechanism();
    push_in(m, 0.25, 60.0, [&] { return m.phase() == contact::DockingPhase::Failed; });
    CHECK(m.phase() == contact::DockingPhase::Failed);
    CHECK(m.failure() == "Überlast");
}

TEST_CASE("Hull sampling: the tanker's whole hull clears the mated depot by ~1 m, 1.5 m closer it collides",
          "[refuelling][docking]") {
    const auto tanker = config::top_stage_config(config::load_vehicle(config_dir, "starship_tanker"));
    const auto depot = config::top_stage_config(config::load_vehicle(config_dir, "starship_depot"));
    const contact::StationCollisionParameters cp;
    const auto points = contact::hull_sample_points(tanker, cp);
    double top = 0.0;
    for (const auto& q : points) top = std::max(top, q.x);
    CHECK(top > 50.0);  // not cut at the dorsal port (x = 20 m)
    const auto t = vehicle::mated_stage_transform(tanker.stages.back().docking_ports.front(),
                                                  depot.stages.back().docking_ports.front());
    const auto shape = contact::station_shape(depot);
    std::vector<Vec3> mated, closer, zero(points.size());
    for (const auto& q : points) {
        mated.push_back(t.apply(q));
        closer.push_back(t.apply(q) - Vec3{0.0, 0.0, 1.5});  // towards the depot axis (its dorsal side is +z)
    }
    CHECK_FALSE(contact::hull_contact(shape, mated, zero, {}, cp).any);
    CHECK(contact::hull_contact(shape, closer, zero, {}, cp).any);
}

TEST_CASE("HLS legs clear the depot when mated (r1 open point)", "[refuelling][docking][hls]") {
    const auto hls = config::top_stage_config(config::load_vehicle(config_dir, "starship_hls"));
    const auto depot = config::top_stage_config(config::load_vehicle(config_dir, "starship_depot"));
    const contact::StationCollisionParameters cp;
    const auto points = contact::hull_sample_points(hls, cp);
    double reach = 0.0;
    for (const auto& q : points) reach = std::max(reach, std::hypot(q.y, q.z));
    CHECK(reach == Catch::Approx(12.0).margin(0.01));  // the leg feet are sampled
    const auto t = vehicle::mated_stage_transform(hls.stages.back().docking_ports.front(),
                                                  depot.stages.back().docking_ports.front());
    std::vector<Vec3> mated, zero(points.size());
    for (const auto& q : points) mated.push_back(t.apply(q));
    const auto contact = contact::hull_contact(contact::station_shape(depot), mated, zero, {}, cp);
    CHECK_FALSE(contact.any);
}

TEST_CASE("Tanker from Waypoint 2 to hard capture at the depot", "[refuelling][docking][proximity]") {
    test::spice_session();
    refuelling_test::TankerAtPort f({20.0, 0.0, 0.0});
    gnc::ProximityParameters p;
    p.start_waypoint = 2;
    REQUIRE(f.session.start_proximity("transfer_dock", p));
    const double t = f.fly([&] { return f.session.docked(); }, 3600.0);
    INFO(f.session.docking_status() << " / " << f.session.proximity_status());
    REQUIRE(f.session.docked());
    CHECK(t < 40.0 * 60.0);
    const auto loads = *f.session.docking_loads();
    CHECK(loads.compression_n < 1.0e5);
    const auto merge = f.session.docking_merge_record();
    CHECK(norm(merge.momentum_after - merge.momentum_before) < 1e-6 * norm(merge.momentum_before));
    const auto& attached = f.session.body(1).vehicle().attached_mass();
    REQUIRE(attached);
    CHECK(attached->mass_kg == Catch::Approx(f.session.body(0).vehicle().mass_properties().mass_kg));
}

TEST_CASE("Tanker at the depot: 0.4 m/s overloads and retreats to Waypoint 2", "[refuelling][docking][proximity]") {
    test::spice_session();
    // inside Waypoint 2 the guidance brakes (0.037 m/s^2): it still hits at ~0.36 m/s, joint damper ~175 kN
    refuelling_test::TankerAtPort f({1.0, 0.0, 0.0}, {-0.40, 0.0, 0.0});
    gnc::ProximityParameters p;
    p.start_waypoint = 2;
    REQUIRE(f.session.start_proximity("transfer_dock", p));
    f.fly([&] { return f.session.docking_phase() == contact::DockingPhase::Failed; }, 120.0, 0.1);
    REQUIRE(f.session.docking_phase() == contact::DockingPhase::Failed);
    f.fly([&] { return f.session.proximity_phase() == gnc::ProximityPhase::Waypoint; }, 1800.0);
    CHECK(f.session.proximity_phase() == gnc::ProximityPhase::Waypoint);
    CHECK_THAT(f.session.proximity_status(), Catch::Matchers::ContainsSubstring("Wegpunkt 2"));
}

TEST_CASE("Tanker undocks from the depot: springs, departure out of the ellipsoid", "[refuelling][docking][slow]") {
    test::spice_session();
    refuelling_test::TankerAtPort f({0.0, 0.0, 0.0});
    // docked at once (ISS i6 scenario start)
    f.session.set_rendezvous_target(1, "transfer_dock", "transfer_dock");
    f.session.dock_in_place();
    REQUIRE(f.session.docked());
    const double chaser_kg = f.session.body(0).vehicle().mass_properties().mass_kg;
    REQUIRE(f.session.undock());
    f.fly([&] { return f.session.departure_phase() == gnc::DeparturePhase::Complete; }, 4.0 * 3600.0, 10.0);
    CHECK(f.session.departure_phase() == gnc::DeparturePhase::Complete);
    const auto sep = f.session.docking_separation_record();
    const double depot_kg = f.session.body(1).vehicle().mass_properties().mass_kg;
    CHECK(sep.relative_speed_mps ==
          Catch::Approx(std::sqrt(2.0 * 150.0 * (1.0 / chaser_kg + 1.0 / depot_kg))).epsilon(0.01));
    CHECK(f.session.docking_status() != "Kollision");
}

namespace {

double tank_kg(const vehicle::FlightBody& body, const std::string& id) {
    for (const auto& k : body.vehicle().stages()[0].tanks) {
        if (k.config.id == id) return k.mass_kg;
    }
    return 0.0;
}

}  // namespace

TEST_CASE("Tank changes of a docked pair move the compound CG, not the hulls", "[refuelling][docking]") {
    test::spice_session();
    refuelling_test::TankerAtPort f({0.0, 0.0, 0.0});
    f.session.set_rendezvous_target(1, "transfer_dock", "transfer_dock");
    f.session.dock_in_place();
    REQUIRE(f.session.docked());
    const auto port_before = vehicle::port_kinematics(f.session.body(1), "transfer_dock").position_gcrf_m;
    const auto chaser_port_before = vehicle::port_kinematics(f.session.body(0), "transfer_dock").position_gcrf_m;
    const Vec3 v_before = f.session.body(1).state().velocity_gcrf_mps;
    const double total_before = f.session.body(1).vehicle().mass_properties().mass_kg;

    // 50 t of LOX from the tanker into the depot (what r4 will do in steps)
    f.session.set_tank_mass(0, 0, "lox_main", tank_kg(f.session.body(0), "lox_main") - 50.0e3);
    f.session.set_tank_mass(1, 0, "lox_main", tank_kg(f.session.body(1), "lox_main") + 50.0e3);

    CHECK(f.session.body(1).vehicle().mass_properties().mass_kg == Catch::Approx(total_before));
    CHECK(norm(vehicle::port_kinematics(f.session.body(1), "transfer_dock").position_gcrf_m - port_before) < 1e-6);
    CHECK(norm(vehicle::port_kinematics(f.session.body(0), "transfer_dock").position_gcrf_m - chaser_port_before) <
          1e-6);
    CHECK(norm(f.session.body(1).state().velocity_gcrf_mps - v_before) < 1e-12);
    CHECK(f.session.body(1).vehicle().attached_mass()->mass_kg ==
          Catch::Approx(f.session.body(0).vehicle().mass_properties().mass_kg));
    REQUIRE(f.session.compound_mass());
    CHECK(f.session.compound_mass()->mass_kg == Catch::Approx(total_before));
}

TEST_CASE("Emptied tanks keep the compound finite", "[refuelling][docking]") {
    test::spice_session();
    refuelling_test::TankerAtPort f({0.0, 0.0, 0.0});
    f.session.set_rendezvous_target(1, "transfer_dock", "transfer_dock");
    f.session.dock_in_place();
    for (const char* tank : {"lox_main", "ch4_main", "lox_header", "ch4_header"}) {
        f.session.set_tank_mass(0, 0, tank, 0.0);
        f.session.set_tank_mass(1, 0, tank, 0.0);
    }
    const auto& attached = f.session.body(1).vehicle().attached_mass();
    REQUIRE(attached);
    CHECK(std::isfinite(attached->position_m.x));
    CHECK(attached->mass_kg == Catch::Approx(f.session.body(0).vehicle().mass_properties().mass_kg));
    CHECK(f.session.compound_mass()->mass_kg ==
          Catch::Approx(f.session.body(1).vehicle().mass_properties().mass_kg));
    CHECK(std::isfinite(f.session.body(1).state().position_gcrf_m.x));
    f.session.advance_to(f.session.epoch() + 60.0);
    CHECK(f.session.docked());
}

TEST_CASE("Proximity start refuses a tanker without propellant or without a thrust axis (review r3 I2)",
          "[refuelling][proximity]") {
    test::spice_session();
    gnc::ProximityParameters p;
    p.start_waypoint = 2;
    {
        refuelling_test::TankerAtPort f({20.0, 0.0, 0.0}, {}, refuelling_test::tanker_config(0.0));
        bool started = true;
        REQUIRE_NOTHROW(started = f.session.start_proximity("transfer_dock", p));
        CHECK_FALSE(started);
        CHECK(f.session.proximity_status() == "Kein Anflug: kein Treibstoff");
        CHECK_FALSE(f.session.docking_phase());  // the target is cleared
    }
    {
        auto no_aft = refuelling_test::tanker_config(200.0e3);
        auto& thrusters = no_aft.stages.back().rcs.thrusters;
        thrusters.erase(std::remove_if(thrusters.begin(), thrusters.end(),
                                       [](const config::RcsThrusterConfig& t) { return t.direction.x < -0.99; }),
                        thrusters.end());
        refuelling_test::TankerAtPort f({20.0, 0.0, 0.0}, {}, no_aft);
        CHECK_FALSE(f.session.start_proximity("transfer_dock", p));
        CHECK_THAT(f.session.proximity_status(), Catch::Matchers::ContainsSubstring("keine Schubrichtung"));
    }
}
