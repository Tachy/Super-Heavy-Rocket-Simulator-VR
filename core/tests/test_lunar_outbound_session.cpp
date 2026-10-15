// The HLS flies the outbound mission on its autopilot: LLO (< 1 km after the trims), NRHO (three revolutions near the
// reference), NRHO -> LLO; time warp, pilot override, refusal and double planning.

#include <algorithm>
#include <cmath>
#include <tuple>
#include <utility>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>

#include <catch2/catch_test_macros.hpp>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/quat.hpp"
#include "starsim/mission/lunar_outbound.hpp"
#include "starsim/mission/lunar_transfer.hpp"
#include "starsim/mission/nrho_reference.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;

namespace {
constexpr double deg = constants::deg_to_rad;
struct Fixture {
    bool spice = (test::spice_session(), true);  // kernels before the members that read them
    spice::SpiceEphemeris ephemeris;
    spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    spice::SpiceOrientation moon = spice::SpiceOrientation::moon_mean_earth();
    environment::PointMassGravity gravity{ephemeris, earth};
    mission::LunarTransferTargeter targeter{gravity, ephemeris, moon};
    time::Epoch t0 = spice::SpiceTimeConverter().from_utc({2026, 11, 2, 0, 0, 0.0});
    sim::FlightSession session{ephemeris, earth, STARSIM_CONFIG_DIR, {}, &moon};
    math::StateVector parking(time::Epoch arrival) const {
        return orbit::state_from_elements({6378137.0 + 200e3, 0.0, 28.5 * deg,
                                           targeter.parking_orbit_raan(arrival, 28.5 * deg).value(), 0.0, 0.0},
                                          ephemeris.gm_m3ps2(ephemeris::Body::Earth));
    }
    void place(time::Epoch arrival) { session.place_in_orbit("starship_hls", parking(arrival), t0); }
    math::StateVector moon_relative_state() {
        const auto& s = session.body(session.controlled_index()).state();
        return mission::moon_relative(ephemeris, {s.position_gcrf_m, s.velocity_gcrf_mps}, session.epoch());
    }
    // Altitude range over the next `revolutions` LLO revolutions (sampled every 20 s).
    std::pair<double, double> llo_range(int revolutions) {
        double low = 1e12, high = -1e12;
        for (int i = 0; i < revolutions * 355; ++i) {
            session.advance_to(session.epoch() + 20.0, 100000);
            const double h = norm(moon_relative_state().position_m) - constants::moon_mean_radius_m;
            low = std::min(low, h);
            high = std::max(high, h);
        }
        return {low, high};
    }
    double mass_kg() const { return session.body(session.controlled_index()).vehicle().mass_properties().mass_kg; }
};
}  // namespace

TEST_CASE("An HLS ticked in the Earth parking orbit does not touch the default site plane", "[sim][lunar][legs]") {
    // The default leg landing site is a ground plane at (0 deg, 0 deg); far from it the plane passes through the orbit
    // (200 km up, ~14 deg away) - the fixed legs must not touch it there.
    Fixture fx;
    fx.place(fx.t0 + 4.0 * 86400.0);
    fx.session.apply_angular_rate(0, {0.0, 0.01, 0.0});  // a slow turn keeps the session ticking
    fx.session.set_attitude_mode({gnc::AttitudeModeKind::Inertial, fx.session.body(0).state().attitude, {}, {}});
    for (int i = 0; i < 60; ++i) {
        fx.session.advance_to(fx.session.epoch() + 100.0, 100000);
        REQUIRE_FALSE(fx.session.body(0).impacted());
    }
}

TEST_CASE("HLS outbound autopilot to the 100 km polar LLO", "[sim][lunar][outbound][validation][slow]") {
    Fixture fx;
    mission::LunarOutboundParameters p;
    p.arrival_epoch = fx.t0 + 4.0 * 86400.0;
    fx.place(p.arrival_epoch);
    const double m0 = fx.mass_kg();
    REQUIRE(fx.session.plan_lunar_outbound(p));
    CHECK_FALSE(fx.session.plan_lunar_outbound(p));  // a flight under way keeps its plan
    // One warp call over the whole transfer (review focus 1): the steps must not be skipped.
    fx.session.advance_to(p.arrival_epoch + 12.0 * 3600.0, 50'000'000);
    INFO(fx.session.lunar_status());
    REQUIRE(fx.session.lunar_step() == sim::LunarStep::InLlo);
    const auto [low, high] = fx.llo_range(3);
    std::printf("[validation] HLS outbound LLO: %.2f x %.2f km, %.0f t used, %s\n", low / 1000.0, high / 1000.0,
                (m0 - fx.mass_kg()) / 1000.0, fx.session.lunar_status().c_str());
    CHECK(low > 99e3);  // spec: 100 km circular within 1 km
    CHECK(high < 101e3);
}

TEST_CASE("HLS outbound autopilot to the NRHO and down to the LLO", "[sim][lunar][outbound][nrho][validation][slow]") {
    Fixture fx;
    mission::LunarOutboundParameters p;
    p.orbit = mission::LunarOrbitKind::Nrho;
    p.arrival_epoch = fx.t0 + 4.5 * 86400.0;
    p.nrho = std::make_shared<mission::NrhoReference>(
        fx.gravity, fx.ephemeris, mission::NrhoSeed{{{1.0221, 0.0, -0.1821}, {0.0, -0.1033, 0.0}}, 9, 2},
        p.arrival_epoch, mission::NrhoReferenceOptions{.revolutions = 6});
    fx.place(p.arrival_epoch);
    REQUIRE(fx.session.plan_lunar_outbound(p));
    // The in-direct transfer joins the NRHO about a day after the swing-by at the arrival perilune.
    fx.session.advance_to(fx.session.lunar_outbound_plan()->insertion.epoch + 2.0 * 3600.0, 50'000'000);
    INFO(fx.session.lunar_status());
    REQUIRE(fx.session.lunar_step() == sim::LunarStep::InNrho);
    double worst_km = 0.0;
    for (int rev = 1; rev <= 3; ++rev) {
        const auto tp = p.nrho->perilune_epochs()[rev];
        fx.session.advance_to(tp, 50'000'000);
        const auto& s = fx.session.body(0).state();
        worst_km = std::max(worst_km, norm(s.position_gcrf_m - p.nrho->state_gcrf(tp).position_m) / 1000.0);
    }
    std::printf("[validation] HLS in the NRHO: worst perilune offset %.2f km over 3 revolutions\n", worst_km);
    CHECK(worst_km < 5.0);  // [E] spec: stays near the reference for three revolutions

    REQUIRE(fx.session.plan_lunar_orbit_transfer(mission::LunarOrbitKind::Llo, p.nrho));
    fx.session.advance_to(fx.session.epoch() + 9.0 * 86400.0, 50'000'000);
    INFO(fx.session.lunar_status());
    REQUIRE(fx.session.lunar_step() == sim::LunarStep::InLlo);
    const auto [low, high] = fx.llo_range(3);
    std::printf("[validation] NRHO -> LLO flown: %.2f x %.2f km\n", low / 1000.0, high / 1000.0);
    CHECK(low > 99e3);
    CHECK(high < 101e3);
}

namespace {
enum class Crew { Autopilot, FliesMcc1, IgnoresMcc1 };
struct PilotFlight {
    double propellant_t = 0.0;
    double mcc2_mps = 0.0;  // 0 when skipped (below the threshold)
    double low_m = 0.0, high_m = 0.0;
    bool in_llo = false;
};

// Lunar l3 Step 3b: the LLO flight with MCC 1 handed to a scripted test pilot (rate commands nulling the cue's attitude
// error, +x translation until 10 % of the planned delta-v is left, then "flown"), or ignored by the crew.
PilotFlight fly_with_crew(Crew crew) {
    Fixture fx;
    mission::LunarOutboundParameters p;
    p.arrival_epoch = fx.t0 + 4.0 * 86400.0;
    p.pilot_corrections = crew != Crew::Autopilot;
    fx.place(p.arrival_epoch);
    const double m0 = fx.mass_kg();
    REQUIRE(fx.session.plan_lunar_outbound(p));
    const auto end = p.arrival_epoch + 12.0 * 3600.0;
    int call = 0;
    bool in_call = false;
    while (end - fx.session.epoch() > 1.0) {
        if (fx.session.lunar_step() != sim::LunarStep::PilotCorrection) {
            in_call = false;
            fx.session.advance_to(std::min(fx.session.epoch() + 300.0, end), 50'000'000);
            continue;
        }
        if (!in_call) ++call;
        in_call = true;
        const auto cue = fx.session.lunar_pilot_cue();
        if (!cue) FAIL("no cue in the step PilotCorrection");
        sim::PilotInput input;
        const double remaining = norm(cue->remaining_delta_v_gcrf_mps);
        if (!(crew == Crew::IgnoresMcc1 && call == 1)) {  // the crew flies every other correction
            if (remaining <= 0.1 * norm(cue->planned.delta_v_gcrf_mps)) {
                fx.session.set_input({});
                fx.session.complete_pilot_correction();
                continue;
            }
            const auto d = fx.session.body(0).state().attitude.inverse_rotate(cue->remaining_delta_v_gcrf_mps / remaining);
            // pitch > 0 turns +x towards +y (rate about +z), yaw > 0 towards -z (rate about +y)
            input.pitch = std::clamp(3.0 * d.y, -1.0, 1.0);
            input.yaw = std::clamp(-3.0 * d.z, -1.0, 1.0);
            if (d.x < 0.0 && std::abs(d.y) < 0.1 && std::abs(d.z) < 0.1) input.pitch = 1.0;  // straight behind
            if (cue->attitude_error_rad < 2.0 * constants::deg_to_rad) input.translation = {1.0, 0.0, 0.0};
        }
        fx.session.set_input(input);
        fx.session.advance_to(fx.session.epoch() + 0.1, 100);
    }
    fx.session.set_input({});
    PilotFlight result;
    int n = 0;  // MCC 2: the second correction commanded (autopilot or crew); 0 when it was below the threshold
    for (const auto& b : fx.session.lunar_burns()) {
        const bool mcc = b.step == sim::LunarStep::MidCourse || b.step == sim::LunarStep::PilotCorrection;
        if (mcc && ++n == 2) result.mcc2_mps = norm(b.node.delta_v_gcrf_mps);
    }
    result.in_llo = fx.session.lunar_step() == sim::LunarStep::InLlo;
    if (result.in_llo) std::tie(result.low_m, result.high_m) = fx.llo_range(1);
    result.propellant_t = (m0 - fx.mass_kg()) / 1000.0;
    return result;
}
}  // namespace

TEST_CASE("Pilot-flown mid-course correction", "[sim][lunar][outbound][pilot][slow]") {
    const auto autopilot = fly_with_crew(Crew::Autopilot);
    const auto flown = fly_with_crew(Crew::FliesMcc1);
    const auto ignored = fly_with_crew(Crew::IgnoresMcc1);
    for (const auto& [name, f] : {std::pair{"autopilot", autopilot}, std::pair{"pilot flies MCC 1", flown},
                                  std::pair{"pilot ignores MCC 1", ignored}}) {
        std::printf("[validation] %s: %.1f t propellant, MCC 2 %.2f m/s, LLO %.2f x %.2f km\n", name, f.propellant_t,
                    f.mcc2_mps, f.low_m / 1000.0, f.high_m / 1000.0);
    }
    for (const auto& f : {flown, ignored}) {
        REQUIRE(f.in_llo);
        CHECK(f.low_m > 99e3);  // spec: 100 km within 1 km after the trims
        CHECK(f.high_m < 101e3);
        CHECK(f.propellant_t > autopilot.propellant_t);
    }
    CHECK(ignored.mcc2_mps > autopilot.mcc2_mps);  // a skipped correction grows the next one
}

namespace {
// A Moon orientation that fails once armed: stands for a planner exception (DOP853 step underflow, ephemeris range).
struct FailingOrientation : frames::BodyOrientation {
    const frames::BodyOrientation& inner;
    bool armed = false;
    explicit FailingOrientation(const frames::BodyOrientation& o) : inner(o) {}
    frames::FixedToInertial fixed_to_gcrf(time::Epoch epoch) const override {
        if (armed) throw std::runtime_error("injected planner failure");
        return inner.fixed_to_gcrf(epoch);
    }
};
}  // namespace

TEST_CASE("A planner exception in the tick aborts the lunar flight", "[sim][lunar][outbound]") {
    // Final review I2: an exception must not escape the tick (the flight would stay stuck on the same due step).
    Fixture fx;
    FailingOrientation failing{fx.moon};
    sim::FlightSession session{fx.ephemeris, fx.earth, STARSIM_CONFIG_DIR, {}, &failing};
    mission::LunarOutboundParameters p;
    p.arrival_epoch = fx.t0 + 4.0 * 86400.0;
    session.place_in_orbit("starship_hls", fx.parking(p.arrival_epoch), fx.t0);
    REQUIRE(session.plan_lunar_outbound(p));
    failing.armed = true;
    REQUIRE_NOTHROW(session.advance_to(session.lunar_outbound_plan()->tli.epoch, 50'000'000));
    CHECK(session.lunar_step() == sim::LunarStep::None);
    CHECK(session.autopilot().state == sim::AutopilotState::Off);
    CHECK(session.lunar_status().find("Planung fehlgeschlagen") != std::string::npos);

    // The planning entry point refuses with a German reason instead of throwing.
    sim::FlightSession second{fx.ephemeris, fx.earth, STARSIM_CONFIG_DIR, {}, &failing};
    second.place_in_orbit("starship_hls", fx.parking(p.arrival_epoch), fx.t0);
    bool planned = true;
    REQUIRE_NOTHROW(planned = second.plan_lunar_outbound(p));
    CHECK_FALSE(planned);
    CHECK(second.lunar_status().find("Planung fehlgeschlagen") != std::string::npos);
    CHECK(second.autopilot().state == sim::AutopilotState::Off);
}

TEST_CASE("Pilot override ends the lunar flight; a short plan is refused", "[sim][lunar][outbound]") {
    Fixture fx;
    mission::LunarOutboundParameters p;
    p.arrival_epoch = fx.t0 + 4.0 * 86400.0;
    fx.place(p.arrival_epoch);
    REQUIRE(fx.session.plan_lunar_outbound(p));
    fx.session.advance_to(fx.session.lunar_outbound_plan()->tli.epoch + 6.0 * 3600.0, 50'000'000);
    INFO(fx.session.lunar_status() << " | " << sim::to_string(fx.session.autopilot().state));
    REQUIRE(fx.session.lunar_step() == sim::LunarStep::Coast);
    fx.session.disengage_autopilot();
    const double m = fx.mass_kg();
    fx.session.advance_to(p.arrival_epoch + 3600.0, 50'000'000);
    CHECK(fx.session.lunar_step() == sim::LunarStep::None);
    CHECK(fx.session.autopilot().state == sim::AutopilotState::Off);
    CHECK(fx.mass_kg() == m);  // no MCC, no LOI fired

    Fixture short_fx;
    // A load that cannot fly the outbound leg (half tanks still give 7.6 km/s on the 120 t HLS): 80 t, 1.9 km/s.
    auto config = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_hls");
    for (auto& tank : config.stages[0].tanks) tank.mass_kg *= 0.05;
    short_fx.session.place_in_orbit(config, short_fx.parking(p.arrival_epoch), short_fx.t0);
    CHECK_FALSE(short_fx.session.plan_lunar_outbound(p));
    REQUIRE(short_fx.session.lunar_outbound_plan() != nullptr);
    CHECK(short_fx.session.lunar_outbound_plan()->reason.find("Treibstoff") != std::string::npos);
    CHECK(short_fx.session.autopilot().state == sim::AutopilotState::Off);
}

namespace {
// Execution-error campaign (l3 Task 8): case i flies the LLO (i < 4) or the NRHO with the TLI realised 1 % short or
// long (sign alternates) and tilted by 0.5 deg about an axis turned by i * 90 deg around the burn direction. The error
// is applied right after the TLI cutoff; the MCCs, insertion and trims must absorb it.
void fly_campaign_case(int i) {
    Fixture fx;
    mission::LunarOutboundParameters p;
    const bool llo = i < 4;
    p.arrival_epoch = fx.t0 + (llo ? 4.0 : 4.5) * 86400.0;
    if (!llo) {
        p.orbit = mission::LunarOrbitKind::Nrho;
        p.nrho = std::make_shared<mission::NrhoReference>(
            fx.gravity, fx.ephemeris, mission::NrhoSeed{{{1.0221, 0.0, -0.1821}, {0.0, -0.1033, 0.0}}, 9, 2},
            p.arrival_epoch, mission::NrhoReferenceOptions{.revolutions = 6});
    }
    fx.place(p.arrival_epoch);
    const double m0 = fx.mass_kg();
    REQUIRE(fx.session.plan_lunar_outbound(p));
    while (fx.session.lunar_step() != sim::LunarStep::Coast) {
        REQUIRE(fx.session.epoch() < p.arrival_epoch);
        fx.session.advance_to(fx.session.epoch() + 10.0, 100000);
    }
    const auto& burns = fx.session.lunar_burns();
    const auto tli = std::find_if(burns.begin(), burns.end(), [](const auto& b) { return b.step == sim::LunarStep::Tli; });
    REQUIRE(tli != burns.end());
    const math::Vec3 dv = tli->node.delta_v_gcrf_mps;
    const math::Vec3 u = normalized(dv);
    const math::Vec3 e1 = normalized(cross(u, std::abs(u.z) < 0.9 ? math::Vec3{0.0, 0.0, 1.0} : math::Vec3{1.0, 0.0, 0.0}));
    const math::Vec3 e2 = cross(u, e1);
    const double phi = i * 90.0 * deg;
    const double scale = (i % 2 ? 1.0 : -1.0) * 0.01;
    const auto tilt = math::Quat::from_axis_angle(std::cos(phi) * e1 + std::sin(phi) * e2, 0.5 * deg);
    fx.session.apply_velocity_error(0, tilt.rotate((1.0 + scale) * dv) - dv);

    double offset_km = 0.0, low = 0.0, high = 0.0;
    if (llo) {
        fx.session.advance_to(p.arrival_epoch + 12.0 * 3600.0, 50'000'000);
        INFO(fx.session.lunar_status());
        REQUIRE(fx.session.lunar_step() == sim::LunarStep::InLlo);
        std::tie(low, high) = fx.llo_range(3);
    } else {
        fx.session.advance_to(fx.session.lunar_outbound_plan()->insertion.epoch + 2.0 * 3600.0, 50'000'000);
        INFO(fx.session.lunar_status());
        REQUIRE(fx.session.lunar_step() == sim::LunarStep::InNrho);
        for (int rev = 1; rev <= 3; ++rev) {
            const auto tp = p.nrho->perilune_epochs()[rev];
            fx.session.advance_to(tp, 50'000'000);
            const auto& s = fx.session.body(0).state();
            offset_km = std::max(offset_km, norm(s.position_gcrf_m - p.nrho->state_gcrf(tp).position_m) / 1000.0);
        }
    }
    std::string mccs;
    for (const auto& b : fx.session.lunar_burns()) {
        if (b.step != sim::LunarStep::MidCourse) continue;
        char buf[32];
        std::snprintf(buf, sizeof buf, "%s%.2f", mccs.empty() ? "" : " / ", norm(b.node.delta_v_gcrf_mps));
        mccs += buf;
    }
    if (mccs.empty()) mccs = "none";
    std::printf("[validation] campaign %d (%s, TLI %+.0f %%, tilt 0.5 deg at %.0f deg): MCC %s m/s, ", i,
                llo ? "LLO" : "NRHO", scale * 100.0, (i % 4) * 90.0, mccs.c_str());
    if (llo) std::printf("LLO %.2f x %.2f km", low / 1000.0, high / 1000.0);
    else std::printf("worst perilune offset %.2f km", offset_km);
    std::printf(", %.0f t used\n", (m0 - fx.mass_kg()) / 1000.0);
    if (llo) {
        CHECK(low > 99e3);  // spec: 100 km circular within 1 km after the trims
        CHECK(high < 101e3);
    } else {
        CHECK(offset_km < 5.0);  // [E] near the reference at the first three perilunes
    }
}
}  // namespace

// One test case per flight so that ctest runs the campaign in parallel.
TEST_CASE("HLS outbound campaign 0", "[sim][lunar][outbound][campaign][validation][slow]") { fly_campaign_case(0); }
TEST_CASE("HLS outbound campaign 1", "[sim][lunar][outbound][campaign][validation][slow]") { fly_campaign_case(1); }
TEST_CASE("HLS outbound campaign 2", "[sim][lunar][outbound][campaign][validation][slow]") { fly_campaign_case(2); }
TEST_CASE("HLS outbound campaign 3", "[sim][lunar][outbound][campaign][validation][slow]") { fly_campaign_case(3); }
TEST_CASE("HLS outbound campaign 4", "[sim][lunar][outbound][campaign][validation][slow]") { fly_campaign_case(4); }
TEST_CASE("HLS outbound campaign 5", "[sim][lunar][outbound][campaign][validation][slow]") { fly_campaign_case(5); }
TEST_CASE("HLS outbound campaign 6", "[sim][lunar][outbound][campaign][validation][slow]") { fly_campaign_case(6); }
TEST_CASE("HLS outbound campaign 7", "[sim][lunar][outbound][campaign][validation][slow]") { fly_campaign_case(7); }
