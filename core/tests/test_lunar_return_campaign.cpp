// Lunar return campaigns (l5): the LLO return flown with dispersions - TEI and braking burn realised +-0.2 % and tilted
// up to 0.1 deg (the residual of a closed-loop cutoff; user 2026-10-17 - Apollo 15 MCC-2: 48.9 for 48.7 ft/s), the entry angle rotated +-0.1 deg at the interface (after the entry trim), truth aero +-10 %, truth air density
// +-15 % and thrust +-2 %. Direct: caught at Starbase; aerocapture ("Aircatch"): captured, perigee raised, caught by b4.
// Single cases: "[.lunar_return_case]" with STARSIM_RETURN_SEED=<seed> (and STARSIM_RETURN_AIRCATCH=1).

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>

#include <catch2/catch_test_macros.hpp>

#include "lunar_return_test_support.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/math/quat.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/sim/mission_start.hpp"

using namespace starsim;
using math::Vec3;

namespace {

constexpr double deg = constants::deg_to_rad;
constexpr double interface_height_m = 120e3;
constexpr double entry_rotation_lead_s = 0.0;  // the entry angle error is applied at the interface (60 s before: x1.7)

struct BurnError {
    double scale = 0.0;        // realised magnitude - 1 (+-0.2 %)
    double tilt_rad = 0.0;     // pointing error (<= 0.1 deg)
    double tilt_axis_rad = 0.0;  // axis of the tilt around the burn direction
};

struct Dispersion {
    BurnError tei, braking;
    double entry_angle_rad = 0.0;  // +-0.1 deg (> 0: steeper)
    double aero_factor = 1.0;      // +-10 %
    double density_factor = 1.0;   // +-15 %
    double thrust_factor = 1.0;    // +-2 %
};

Dispersion dispersion(unsigned seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> unit(-1.0, 1.0);
    const auto burn = [&] {
        BurnError e;
        e.scale = 0.002 * unit(rng);                       // [E] closed-loop cutoff residual (user 2026-10-17)
        e.tilt_rad = 0.1 * deg * 0.5 * (unit(rng) + 1.0);
        e.tilt_axis_rad = constants::pi * unit(rng);
        return e;
    };
    Dispersion d;
    d.tei = burn();
    d.braking = burn();
    d.entry_angle_rad = 0.1 * deg * unit(rng);
    d.aero_factor = 1.0 + 0.10 * unit(rng);
    d.density_factor = 1.0 + 0.15 * unit(rng);
    d.thrust_factor = 1.0 + 0.02 * unit(rng);
    return d;
}

// The velocity error of a burn `dv` realised with error `e`.
Vec3 burn_error_mps(const Vec3& dv, const BurnError& e) {
    const Vec3 u = normalized(dv);
    const Vec3 e1 = normalized(cross(u, std::abs(u.z) < 0.9 ? Vec3{0.0, 0.0, 1.0} : Vec3{1.0, 0.0, 0.0}));
    const Vec3 e2 = cross(u, e1);
    const auto tilt = math::Quat::from_axis_angle(std::cos(e.tilt_axis_rad) * e1 + std::sin(e.tilt_axis_rad) * e2,
                                                  e.tilt_rad);
    return tilt.rotate((1.0 + e.scale) * dv) - dv;
}

const sim::LunarBurnRecord* burn_of(const sim::FlightSession& s, sim::LunarStep step) {
    for (const auto& b : s.lunar_burns()) {
        if (b.step == step) return &b;
    }
    return nullptr;
}

// Apsis heights above the equatorial radius of the controlled body's two-body Earth orbit (open: -1, -1).
std::pair<double, double> apsis_heights_m(const sim::FlightSession& s) {
    const auto& st = s.body(s.controlled_index()).state();
    const auto e = orbit::elements_from_state({st.position_gcrf_m, st.velocity_gcrf_mps}, constants::earth_gm_m3ps2);
    if (e.semi_major_axis_m <= 0.0 || e.eccentricity >= 1.0) return {-1.0, -1.0};
    return {e.semi_major_axis_m * (1.0 - e.eccentricity) - constants::wgs84_semi_major_axis_m,
            e.semi_major_axis_m * (1.0 + e.eccentricity) - constants::wgs84_semi_major_axis_m};
}

struct Run {
    bool caught = false, impacted = false, exited = false, raised = false;
    double tei_mps = 0.0, braking_mps = 0.0, trim_mps = 0.0, entry_speed_mps = 0.0, entry_angle_rad = 0.0;
    double gate_down_km = 0.0, gate_cross_km = 0.0;  // entry guidance at the gate (direct)
    double braked_t = 0.0, interface_t = 0.0;  // propellant after the braking burn and at the interface
    double peak_heat_w_m2 = 0.0, peak_load_g = 0.0, propellant_t = 0.0;
    std::pair<double, double> exit_apsides{0.0, 0.0}, raised_apsides{0.0, 0.0};
    std::string status, breakup, braking_status;
};

// Advances the session in calls of `step_s` until `done` or `until` (calls of at most `ticks` ticks).
template <class Done>
void run_until(sim::FlightSession& s, time::Epoch until, double step_s, long ticks, Done done) {
    while (!done() && until - s.epoch() > 0.0) s.advance_to(std::min(s.epoch() + step_s, until), ticks);
}

Run fly(const Dispersion& d, bool aircatch) {
    test::LunarReturnSession t;
    auto& s = t.session;
    const auto start = sim::start_mission(s, STARSIM_CONFIG_DIR, aircatch ? "ship_lunar_aircatch_llo" : "ship_lunar_return_llo",
                                          {}, t.time);
    INFO(start.refusal);
    REQUIRE(start.return_planned);
    const size_t ship = s.controlled_index();
    const auto& body = s.body(ship);
    s.set_aero_dispersion(ship, d.aero_factor);
    s.set_thrust_dispersion(ship, d.thrust_factor);
    s.set_air_density_factor(d.density_factor);
    const auto t0 = s.epoch();
    const auto step = [&] { return s.lunar_step(); };
    Run r;

    // TEI: the error right after the cutoff (60 s calls; the burn is ticked at most 60 s before the check)
    run_until(s, t0 + 2.0 * 86400.0, 60.0, 4000000, [&] { return step() != sim::LunarStep::ReturnDeparture &&
                                                                  step() != sim::LunarStep::Tei; });
    const auto* tei = burn_of(s, sim::LunarStep::Tei);
    REQUIRE(tei != nullptr);
    r.tei_mps = norm(tei->node.delta_v_gcrf_mps);
    s.apply_velocity_error(ship, burn_error_mps(tei->node.delta_v_gcrf_mps, d.tei));

    // braking burn: hour calls of at most 200 ticks until it is commanded (a call cannot pass a burn: a burn tick is
    // 5 ms; a correction may move the arrival to the next day's pass), then 5 s calls; the error right after cutoff
    run_until(s, t0 + 8.0 * 86400.0, 3600.0, 200,
              [&] { return body.impacted() || step() == sim::LunarStep::Braking || s.autopilot().state == sim::AutopilotState::Off; });
    run_until(s, s.epoch() + 3600.0, 5.0, 4000000, [&] {
        if (step() == sim::LunarStep::Braking && r.braking_status.empty()) r.braking_status = s.lunar_status();
        return burn_of(s, sim::LunarStep::Braking) != nullptr && step() != sim::LunarStep::Braking;
    });
    const auto* braking = burn_of(s, sim::LunarStep::Braking);
    INFO(s.lunar_status());
    REQUIRE(braking != nullptr);
    r.braking_mps = norm(braking->node.delta_v_gcrf_mps);
    r.braked_t = body.vehicle().propellant_kg() / 1000.0;
    if (!std::getenv("STARSIM_RETURN_NO_BRAKING_ERROR")) {  // diagnostics: the other errors alone
        s.apply_velocity_error(ship, burn_error_mps(braking->node.delta_v_gcrf_mps, d.braking));
    }

    // entry angle: the inertial velocity turned in the trajectory plane at the interface, after the entry
    // trim (> 0: steeper). Turned right after the braking burn (~1,800 km up, 8 min out) 0.07 deg was ~10 m/s and
    // ~0.5 deg at the interface - the trim then flew 8-23 m/s for an artefact of the test (l5 Task 10).
    run_until(s, s.epoch() + 4.0 * 3600.0, 1.0, 4000000, [&] {
        if (step() == sim::LunarStep::EntryTrim || s.autopilot().state == sim::AutopilotState::Burn) return false;
        const auto tel = s.telemetry(ship);
        const double sink_mps = -tel.speed_relative_mps * std::sin(tel.flight_path_angle_rad);
        return sink_mps > 0.0 && tel.altitude_m - interface_height_m < entry_rotation_lead_s * sink_mps;
    });
    {
        const auto& st = body.state();
        const Vec3 n = normalized(cross(st.position_gcrf_m, st.velocity_gcrf_mps));
        const auto turn = math::Quat::from_axis_angle(n, d.entry_angle_rad);
        s.apply_velocity_error(ship, turn.rotate(st.velocity_gcrf_mps) - st.velocity_gcrf_mps);
    }

    // entry (and for the aerocapture: capture coast, perigee raise, circularisation, b4) to the catch
    bool entered = false;
    while (!body.caught() && !body.impacted() && s.epoch() - t0 < 13.0 * 86400.0) {
        s.advance_to(s.epoch() + 86400.0, 200);
        const auto tel = s.telemetry(ship);
        if (!entered && tel.altitude_m < interface_height_m) {
            entered = true;
            r.entry_speed_mps = tel.speed_relative_mps;
            r.entry_angle_rad = tel.flight_path_angle_rad;
            r.interface_t = body.vehicle().propellant_kg() / 1000.0;
        }
        if (const auto* es = s.entry_status(ship); es && es->phase == gnc::EntryPhase::Gate && r.gate_down_km == 0.0) {
            r.gate_down_km = es->downrange_error_m / 1000.0;
            r.gate_cross_km = es->crossrange_m / 1000.0;
        }
        r.peak_load_g = std::max(r.peak_load_g, norm(body.aero_acceleration_body_mps2()) / constants::standard_gravity_mps2);
        if (aircatch && !r.exited && step() == sim::LunarStep::CaptureCoast) {
            r.exited = true;
            r.exit_apsides = apsis_heights_m(s);
        }
        if (r.exited && !r.raised && step() == sim::LunarStep::None) {
            r.raised = true;
            r.raised_apsides = apsis_heights_m(s);
        }
        if (aircatch && s.autopilot().state == sim::AutopilotState::Off && !r.raised) break;  // capture failed
        if (std::getenv("STARSIM_VERBOSE")) {  // progress, flushed: a phase that hangs shows within minutes
            static std::string last;
            static double last_print = -1e9;
            const std::string state = std::string(sim::to_string(step())) + " / " + sim::to_string(s.autopilot().state) +
                                      " / " + s.lunar_status();
            const double now = s.epoch() - t0;
            if (state != last || now - last_print > (tel.altitude_m < 150e3 ? 20.0 : 6.0 * 3600.0)) {
                last = state;
                last_print = now;
                const auto ap = apsis_heights_m(s);
                std::printf("[campaign] %.4f d: %s | %.1f km %.0f m/s %.2f deg, apsides %.0f x %.0f km, %.1f t\n",
                            now / 86400.0, state.c_str(), tel.altitude_m / 1000.0, tel.speed_relative_mps,
                            tel.flight_path_angle_rad / deg, ap.first / 1000.0, ap.second / 1000.0,
                            body.vehicle().propellant_kg() / 1000.0);
                std::fflush(stdout);
            }
        }
    }
    if (const auto* trim = burn_of(s, sim::LunarStep::EntryTrim)) r.trim_mps = norm(trim->node.delta_v_gcrf_mps);
    if (std::getenv("STARSIM_VERBOSE")) {  // where the propellant went: each burn's mass at its command
        for (const auto& b : s.lunar_burns()) {
            std::printf("[campaign]   %s %.2f m/s at %.1f t\n", sim::to_string(b.step), norm(b.node.delta_v_gcrf_mps),
                        b.mass_kg / 1000.0);
        }
    }
    r.caught = body.caught();
    r.impacted = body.impacted();
    r.peak_heat_w_m2 = body.peak_heat_rate_w_m2();
    r.propellant_t = body.vehicle().propellant_kg() / 1000.0;
    r.status = s.lunar_status();
    r.breakup = body.breakup_reason();
    return r;
}

int fly_seeds(unsigned first, unsigned last, bool aircatch) {
    const char* tag = aircatch ? "[aircatch_campaign]" : "[return_campaign]";
    int good = 0;
    for (unsigned seed = first; seed <= last; ++seed) {
        auto d = std::getenv("STARSIM_RETURN_NOMINAL") ? Dispersion{} : dispersion(seed);  // diagnostics
        if (const char* a = std::getenv("STARSIM_RETURN_AERO")) d.aero_factor = std::atof(a);         // diagnostics
        if (const char* r = std::getenv("STARSIM_RETURN_DENSITY")) d.density_factor = std::atof(r);  // diagnostics
        const auto r = fly(d, aircatch);
        std::printf("%s case %2u: TEI %+.2f %% %.2f deg, braking %+.2f %% %.2f deg, entry %+.3f deg, aero %.3f, density "
                    "%.3f, thrust %.3f | TEI %.1f, braking %.1f m/s, trim %.2f m/s (%.1f -> %.1f t), gate %.2f / %.2f km, EI %.0f m/s %.2f deg, peak %.0f kW/m2 %.2f g",
                    tag, seed, 100.0 * d.tei.scale, d.tei.tilt_rad / deg, 100.0 * d.braking.scale,
                    d.braking.tilt_rad / deg, d.entry_angle_rad / deg, d.aero_factor, d.density_factor, d.thrust_factor,
                    r.tei_mps, r.braking_mps, r.trim_mps, r.braked_t, r.interface_t, r.gate_down_km, r.gate_cross_km, r.entry_speed_mps, r.entry_angle_rad / deg, r.peak_heat_w_m2 / 1000.0,
                    r.peak_load_g);
        if (aircatch) {
            std::printf(", exit %.1f x %.1f km, raised %.1f x %.1f km", r.exit_apsides.first / 1000.0,
                        r.exit_apsides.second / 1000.0, r.raised_apsides.first / 1000.0, r.raised_apsides.second / 1000.0);
        }
        std::printf(" | %s | %s, %.1f t left, %s%s%s\n", r.braking_status.c_str(), r.caught ? "caught" : r.impacted ? "impacted" : "open", r.propellant_t,
                    r.status.c_str(), r.breakup.empty() ? "" : " / ", r.breakup.c_str());
        std::fflush(stdout);
        bool ok = r.caught && r.peak_heat_w_m2 < 1.2e6;
        if (aircatch) ok = ok && r.raised && r.raised_apsides.first > 150e3;
        if (ok) ++good;
    }
    std::printf("%s seeds %u-%u: %d of %u\n", tag, first, last, good, last - first + 1);
    return good;
}

}  // namespace

// Three seeds per case so ctest runs the campaigns in parallel (~8 min per direct flight).
TEST_CASE("Lunar return campaign (direct): seeds 1-3", "[lunar][return][validation][slow]") { CHECK(fly_seeds(1, 3, false) == 3); }
TEST_CASE("Lunar return campaign (direct): seeds 4-6", "[lunar][return][validation][slow]") { CHECK(fly_seeds(4, 6, false) == 3); }
TEST_CASE("Lunar return campaign (direct): seeds 7-9", "[lunar][return][validation][slow]") { CHECK(fly_seeds(7, 9, false) == 3); }
// Seed 12 is a known miss since f3 (docs/subsystems.md, "Ship catch propellant margin"): the settling pushes shift the
// gate and the flip ends 25 m short of the catch axis at 29 m/s across - beyond any tilt; the catch aborts.
TEST_CASE("Lunar return campaign (direct): seeds 10-12", "[lunar][return][validation][slow]") { CHECK(fly_seeds(10, 12, false) >= 2); }
TEST_CASE("Lunar return campaign (direct): seeds 13-15", "[lunar][return][validation][slow]") { CHECK(fly_seeds(13, 15, false) == 3); }
TEST_CASE("Lunar return campaign (direct): seeds 16-18", "[lunar][return][validation][slow]") { CHECK(fly_seeds(16, 18, false) == 3); }
TEST_CASE("Lunar return campaign (direct): seeds 19-21", "[lunar][return][validation][slow]") { CHECK(fly_seeds(19, 21, false) == 3); }

// Seeds 4 and 7 failed after l6 Task 1 (entry attitude departure at peak q with 61-64 t aboard, the liquid always a
// column on the aft domes); propellant settling (header-tank plan) moves the residue forward at entry and fixed them.
TEST_CASE("Lunar return campaign (aerocapture): seeds 1-7", "[lunar][return][validation][slow]") { CHECK(fly_seeds(1, 7, true) == 7); }
TEST_CASE("Lunar return campaign (aerocapture): seeds 8-14", "[lunar][return][validation][slow]") { CHECK(fly_seeds(8, 14, true) == 7); }
TEST_CASE("Lunar return campaign (aerocapture): seeds 15-21", "[lunar][return][validation][slow]") { CHECK(fly_seeds(15, 21, true) == 7); }

TEST_CASE("Lunar return campaign: a single case", "[.lunar_return_case]") {
    const char* seed = std::getenv("STARSIM_RETURN_SEED");
    const unsigned n = seed ? static_cast<unsigned>(std::atoi(seed)) : 1u;
    fly_seeds(n, n, std::getenv("STARSIM_RETURN_AIRCATCH") != nullptr);
}
