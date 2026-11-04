// Ship flip and landing parameters from the vehicle config (plan 2026-11-03 planner-vehicle-parameters Task 3): K values
// read, D values equal to their hand formulas, refusal without the keys, inheritance by reused ship stages.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/gnc/descent_planner.hpp"
#include "starsim/gnc/ship_landing.hpp"
#include "starsim/gnc/vehicle_capability.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/propulsion/engine_performance.hpp"

using namespace starsim;
using Catch::Approx;

namespace {

constexpr double deg = constants::deg_to_rad;
constexpr double g0 = constants::standard_gravity_mps2;
constexpr double p0 = constants::standard_pressure_pa;

const config::StageConfig& stage_of(const config::VehicleConfig& cfg, const std::string& id) {
    const auto it = std::find_if(cfg.stages.begin(), cfg.stages.end(), [&](const auto& s) { return s.id == id; });
    REQUIRE(it != cfg.stages.end());
    return *it;
}

// Fresnel integrals C(x) = int_0^x cos(pi s^2 / 2) ds, S(x) = int_0^x sin(pi s^2 / 2) ds by their power series.
double fresnel_c(double x) {
    double sum = 0.0;
    for (int n = 0; n < 12; ++n) {
        sum += std::pow(-1.0, n) * std::pow(constants::pi / 2.0, 2 * n) * std::pow(x, 4 * n + 1) /
               (std::tgamma(2.0 * n + 1.0) * (4 * n + 1));
    }
    return sum;
}
double fresnel_s(double x) {
    double sum = 0.0;
    for (int n = 0; n < 12; ++n) {
        sum += std::pow(-1.0, n) * std::pow(constants::pi / 2.0, 2 * n + 1) * std::pow(x, 4 * n + 3) /
               (std::tgamma(2.0 * n + 2.0) * (4 * n + 3));
    }
    return sum;
}

}  // namespace

// Hand formulas of the derivations, independent of the config.
TEST_CASE("Ship landing derivations follow their hand formulas", "[ship][landing][config]") {
    // 3 -> 2 window: outgoing set 3 MN at minimum on a 250 t ship decelerates 12 - g0 while it shuts down for 0.5 s;
    // the incoming 6 MN at 95 % stops 300 t within 500 m from sqrt(2 (19 - g0) 500).
    gnc::EngineSetThrust out{3.0e6, 9.0e6, 15.0 * deg, 0.35, 20.0, 0.3, 1.0};
    gnc::EngineSetThrust in{2.0e6, 6.0e6, 15.0 * deg, 0.35, 20.0, 0.3, 1.0};
    auto w = gnc::ship_switch_sink_window(out, in, 250.0e3, 300.0e3, 500.0, 0.05, 0.5, g0);
    CHECK(w.min_sink_mps == Approx((12.0 - g0) * 0.5).epsilon(1e-12));
    CHECK(w.max_sink_mps == Approx(std::sqrt(2.0 * (19.0 - g0) * 500.0)).epsilon(1e-12));
    // an outgoing set that cannot exceed the weight needs no minimum
    out.min_n = 2.0e6;
    w = gnc::ship_switch_sink_window(out, in, 250.0e3, 300.0e3, 500.0, 0.05, 0.5, g0);
    CHECK(w.min_sink_mps == 0.0);

    // Flip push without spool-up: rest-to-rest 90 deg at alpha, theta = alpha t^2 / 2 to the middle (x = sqrt(1/2) in
    // Fresnel terms for any alpha): dv = a sqrt(pi / alpha) (C + S), travel = a (T sqrt(pi / alpha) C + (1 - sqrt 2) / alpha).
    const double a = 20.0, alpha = 0.5, brake = 10.0, tilt = 20.0 * deg;
    gnc::FlipModel model{a, alpha, 0.0, brake, 0.0, 0.0, tilt};  // no tilt-in authority: braking at the full tilt
    const auto f = gnc::ship_flip_push(model);
    const double t_f = std::sqrt(2.0 * constants::pi / alpha);
    const double x = std::sqrt(0.5), c = fresnel_c(x), s = fresnel_s(x);
    CHECK(f.duration_s == Approx(t_f).epsilon(1e-12));
    CHECK(f.speed_mps == Approx(a * std::sqrt(constants::pi / alpha) * (c + s)).epsilon(1e-6));
    CHECK(f.travel_m == Approx(a * (t_f * std::sqrt(constants::pi / alpha) * c + (1.0 - std::sqrt(2.0)) / alpha)).epsilon(1e-6));
    CHECK(f.tilt_in_m == 0.0);
    CHECK(f.brake_m == Approx(f.speed_mps * f.speed_mps / (2.0 * brake * std::sin(tilt))).epsilon(1e-12));
    CHECK(f.lead_m() == Approx(f.travel_m + f.brake_m).epsilon(1e-12));

    // Tilt-in, rest-to-rest through phi at alpha_b without a rate cap (T = 2 sqrt(phi / alpha_b)): the horizontal
    // braking a sin(theta) removes dv_b = a sqrt(pi / alpha_b) (S(y) + sin(phi) C(y) - cos(phi) S(y)), y = sqrt(phi / pi);
    // a binding rate cap makes the turn last phi / rate + rate / alpha_b.
    const double alpha_b = 0.6;
    model.brake_alpha_radps2 = alpha_b;
    const auto t = gnc::ship_flip_push(model);
    const double y = std::sqrt(tilt / constants::pi);
    const double dv_b = brake * std::sqrt(constants::pi / alpha_b) *
                        (fresnel_s(y) + std::sin(tilt) * fresnel_c(y) - std::cos(tilt) * fresnel_s(y));
    CHECK(t.tilt_in_s == Approx(2.0 * std::sqrt(tilt / alpha_b)).epsilon(1e-12));
    CHECK(t.brake_speed_mps == Approx(t.speed_mps - dv_b).epsilon(1e-6));
    CHECK(t.tilt_in_m < t.speed_mps * t.tilt_in_s);
    CHECK(t.tilt_in_m > t.brake_speed_mps * t.tilt_in_s);
    CHECK(t.brake_m == Approx(t.brake_speed_mps * t.brake_speed_mps / (2.0 * brake * std::sin(tilt))).epsilon(1e-12));
    CHECK(t.lead_m() == Approx(t.travel_m + t.tilt_in_m + t.brake_m).epsilon(1e-12));
    model.brake_rate_max_radps = 0.2;
    CHECK(gnc::ship_flip_push(model).tilt_in_s == Approx(tilt / 0.2 + 0.2 / alpha_b).epsilon(1e-12));

    // Spool-up over T_s: the authority ramps, the flip lasts longer (by less than T_s); a vanishing spool is the
    // closed form above.
    model.brake_rate_max_radps = 0.0;
    model.brake_alpha_radps2 = 0.0;
    model.spool_s = 3.0;
    const auto spooled = gnc::ship_flip_push(model);
    CHECK(spooled.duration_s > t_f);
    CHECK(spooled.duration_s < t_f + model.spool_s);
    model.spool_s = 1e-9;
    CHECK(gnc::ship_flip_push(model).duration_s == Approx(t_f).epsilon(1e-6));
    CHECK(gnc::ship_flip_push(model).speed_mps == Approx(f.speed_mps).epsilon(1e-6));
    // ... and the spooled turn's switch: with the authority ramping t / T_s over the whole flip (T_s beyond the end),
    // theta = alpha t^3 / (6 T_s) up to the switch t_s, the rate zero at t_e = sqrt(2) t_s and there
    // theta(t_e) = alpha / T_s (t_s^3 / 3 + t_s^2 (t_e - t_s) - t_e^3 / 6) = pi / 2.
    model.spool_s = 100.0;
    const auto slow = gnc::ship_flip_push(model);
    const double t_s = slow.duration_s / std::sqrt(2.0);
    const double end_angle = alpha / model.spool_s *
                             (t_s * t_s * t_s / 3.0 + t_s * t_s * (slow.duration_s - t_s) -
                              slow.duration_s * slow.duration_s * slow.duration_s / 6.0);
    CHECK(slow.duration_s < model.spool_s);
    CHECK(end_angle == Approx(0.5 * constants::pi).epsilon(1e-9));

    // Rate limit: sqrt(alpha tilt) below the gimbal slew rate, else the slew rate.
    gnc::EngineSetThrust set{3.0e6, 9.0e6, 15.0 * deg, 20.0 * deg, 20.0, 0.3, 1.0};
    const double inertia = 5.0e7;
    const double alpha_min = 3.0e6 * std::sin(15.0 * deg) * 20.0 / inertia;
    CHECK(gnc::ship_phase_rate_limit_radps(set, 20.0 * deg, inertia) == Approx(std::sqrt(alpha_min * 20.0 * deg)));
    set.gimbal_rate_radps = 0.1;
    CHECK(gnc::ship_phase_rate_limit_radps(set, 20.0 * deg, inertia) == 0.1);
}

// Task 3 fix round 2 (I2): the gate's horizontal speed v0 along the line enters the lead; the in-flight re-aim uses it.
TEST_CASE("Gate lead follows the gate's horizontal speed", "[ship][landing][config]") {
    const double a = 20.0, alpha = 0.5, brake = 10.0, tilt = 20.0 * deg, v0 = 5.0;
    const double t_f = std::sqrt(2.0 * constants::pi / alpha);
    // Hand case, no tilt-in authority: v0 travels through the flip, the braking starts at dv + v0:
    // lead = travel(0) + v0 T + (dv + v0)^2 / (2 a sin(tilt)).
    gnc::FlipModel model{a, alpha, 0.0, brake, 0.0, 0.0, tilt};
    const auto f0 = gnc::ship_flip_push(model);
    model.initial_speed_mps = v0;
    const auto f5 = gnc::ship_flip_push(model);
    CHECK(f5.duration_s == f0.duration_s);
    CHECK(f5.speed_mps == f0.speed_mps);
    CHECK(f5.lead_m() ==
          Approx(f0.travel_m + v0 * t_f + (f0.speed_mps + v0) * (f0.speed_mps + v0) / (2.0 * brake * std::sin(tilt)))
              .epsilon(1e-12));
    // With the tilt-in: its speed loss does not depend on the speed, so
    // lead(v0) - lead(0) = v0 (T + t_tilt) + ((v_b + v0)^2 - v_b^2) / (2 a sin(tilt)), v_b the brake speed at v0 = 0.
    model.brake_alpha_radps2 = 0.6;
    model.initial_speed_mps = 0.0;
    const auto t0 = gnc::ship_flip_push(model);
    model.initial_speed_mps = v0;
    const auto t5 = gnc::ship_flip_push(model);
    const double vb = t0.brake_speed_mps;
    CHECK(t5.brake_speed_mps == Approx(vb + v0).epsilon(1e-12));
    CHECK(t5.lead_m() - t0.lead_m() ==
          Approx(v0 * (t0.duration_s + t0.tilt_in_s) + ((vb + v0) * (vb + v0) - vb * vb) / (2.0 * brake * std::sin(tilt)))
              .epsilon(1e-9));
    // Moving away from the tower: the tilt-in travel ends at rest (no reverse travel). From 0.5 m/s at the flip's end,
    // early in the tilt-in (theta = alpha_b t^2 / 2, sin theta ~ theta): v = 0.5 - a alpha_b t^3 / 6 reaches zero at
    // t* = (3 / (a alpha_b))^(1/3) after (3/4) 0.5 t*.
    model.initial_speed_mps = 0.5 - f0.speed_mps;
    const auto stop = gnc::ship_flip_push(model);
    const double t_stop = std::cbrt(3.0 / (brake * 0.6));
    CHECK(stop.brake_speed_mps == 0.0);
    CHECK(stop.brake_m == 0.0);
    CHECK(stop.tilt_in_m == Approx(0.75 * 0.5 * t_stop).epsilon(0.02));
    model.initial_speed_mps = -1.0 - f0.speed_mps;  // already backwards at the flip's end: nothing after the flip
    const auto back = gnc::ship_flip_push(model);
    CHECK(back.tilt_in_m == 0.0);
    CHECK(back.lead_m() == back.travel_m);

    // ship_gate_lead_m: the parameters' model with v0; half the mass doubles the linear accelerations (same thrust).
    gnc::ShipLandingParameters p;
    p.gate_flip = gnc::FlipModel{a, alpha, 0.0, brake, 0.6, 0.0, tilt};
    p.gate_flip_mass_kg = 200.0e3;
    gnc::FlipModel light = p.gate_flip;
    light.flip_accel_mps2 *= 2.0;
    light.brake_accel_mps2 *= 2.0;
    light.initial_speed_mps = v0;
    CHECK(gnc::ship_gate_lead_m(p, v0, 100.0e3) == Approx(gnc::ship_flip_push(light).lead_m()).epsilon(1e-12));
    CHECK(gnc::ship_gate_lead_m(p, v0, 200.0e3) == Approx(t5.lead_m()).epsilon(1e-12));

    // From a flown flip (fix round 3): a flip with the analytic model's numbers gives the analytic lead; another gate
    // speed carries on through it, (v0 - v0_flip) T, and enters the brake.
    gnc::ShipFlipPrediction flown;
    flown.reached = true;
    flown.duration_s = t0.duration_s;
    flown.gate_speed_mps = 0.0;
    flown.travel_m = t0.travel_m;
    flown.speed_mps = t0.speed_mps;
    CHECK(gnc::ship_gate_lead_m(p, flown, 0.0, 200.0e3) == Approx(t0.lead_m()).epsilon(1e-12));
    CHECK(gnc::ship_gate_lead_m(p, flown, v0, 200.0e3) == Approx(t5.lead_m()).epsilon(1e-12));
    const auto brake_only = gnc::ship_brake_after_flip(p.gate_flip, t5.speed_mps + v0);
    CHECK(brake_only.tilt_in_m == Approx(t5.tilt_in_m).epsilon(1e-12));
    CHECK(brake_only.brake_m == Approx(t5.brake_m).epsilon(1e-12));
}

// Task 3 fix (I1): the first plan's guess ends a phase from the state, not in the middle of a wide derived window.
TEST_CASE("Descent guess switches as late as the later sets can finish", "[ship][landing][scvx]") {
    gnc::DescentModel model;
    model.engine_sets[0].thrust_max_n = 9.0e6;
    model.engine_sets[1].thrust_max_n = 6.0e6;
    model.engine_sets[2].thrust_max_n = 6.0e6;
    model.gravity_mps2 = {-g0, 0.0, 0.0};
    gnc::DescentTarget target;
    target.position_m = {0.0, 0.0, 0.0};
    target.velocity_mps = {-1.0, 0.0, 0.0};
    gnc::DescentLimits limits;
    limits.switches = {{{0.0, 0.0, 0.0, 1000.0}, {0.0, 0.0, 0.0, 1000.0}}};
    limits.min_phase_s = {1.0, 1.0, 1.0};
    const auto state = [](double height_m) {
        return gnc::make_state(300.0e3, {height_m, 0.0, 0.0}, {-80.0, 0.0, 0.0}, math::Quat{1.0, 0.0, 0.0, 0.0}, {});
    };
    // guess decelerations at 0.8 of full thrust: 24 - g0 (three), 16 - g0 (pair)
    const double d0 = 24.0 - g0, d1 = 16.0 - g0;
    // 500 m up: the pair's two minimum phases remove 2 d1 above the final 1 m/s; the three reach that within the height
    auto ref = gnc::initial_guess(model, state(500.0), 0, target, limits, 5);
    const double late = 1.0 + 2.0 * d1;
    CHECK(ref.duration_s[0] == Approx((80.0 - late) / d0).epsilon(1e-12));
    CHECK(ref.duration_s[1] == Approx(1.0).epsilon(1e-12));
    CHECK(ref.duration_s[2] == Approx(1.0).epsilon(1e-12));
    // 150 m up: the three cannot brake below sqrt(80^2 - 2 d0 150) within the height
    ref = gnc::initial_guess(model, state(150.0), 0, target, limits, 5);
    const double reach = std::sqrt(80.0 * 80.0 - 2.0 * d0 * 150.0);
    CHECK(ref.duration_s[0] == Approx((80.0 - reach) / d0).epsilon(1e-12));
    // within the window
    limits.switches[0].max_sink_mps = 10.0;
    ref = gnc::initial_guess(model, state(500.0), 0, target, limits, 5);
    CHECK(ref.duration_s[0] == Approx((80.0 - 10.0) / d0).epsilon(1e-12));
    // a window above the speed: no switch to make, phase 0 only its minimum
    limits.switches[0] = {0.0, 0.0, 100.0, 1000.0};
    ref = gnc::initial_guess(model, state(500.0), 0, target, limits, 5);
    CHECK(ref.duration_s[0] == 1.0);
    // the planner's second start: the window's middle
    limits.switches[0] = {0.0, 0.0, 20.0, 40.0};
    ref = gnc::initial_guess(model, state(500.0), 0, target, limits, 5, gnc::SwitchGuess::WindowMiddle);
    CHECK(ref.duration_s[0] == Approx((80.0 - 30.0) / d0).epsilon(1e-12));
}

TEST_CASE("Ship landing parameters follow the vehicle config", "[ship][landing][config]") {
    const auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    const auto& ship = stage_of(cfg, "ship");
    REQUIRE(ship.ship_landing.has_value());
    const auto p = gnc::ship_landing_parameters(cfg, ship);

    // K: the config's values
    CHECK(p.flip_groups == std::vector<std::string>{"ship/sea_level_1", "ship/sea_level_2", "ship/sea_level_3"});
    CHECK(p.scvx.snapshot.phase_groups[0] == p.flip_groups);
    CHECK(p.scvx.snapshot.phase_groups[1] == std::vector<std::string>{"ship/sea_level_1", "ship/sea_level_2"});
    CHECK(p.scvx.snapshot.phase_groups[2] == p.scvx.snapshot.phase_groups[1]);
    CHECK(p.flip_throttle == 0.8);
    CHECK(p.flip_tolerance_rad == 15.0 * deg);
    CHECK(p.flip_settled_rate_radps == 0.2);
    CHECK(p.flip_flaps_until_tilt_rad == 30.0 * deg);
    CHECK(p.descent_propellant_margin == 1.3);
    CHECK(p.scvx.lateral_effort_fraction == 0.03);
    CHECK(p.scvx.snapshot.reserve_kg == 1000.0);  // [stage.tanks] unusable header propellant
    CHECK(ship.unusable_propellant.main_kg == 1000.0);
    const auto& l = p.scvx.settings.limits;
    CHECK(l.tilt_max_rad[0] == 20.0 * deg);
    CHECK(l.tilt_max_rad[1] == 20.0 * deg);
    CHECK(l.tilt_max_rad[2] == 15.0 * deg);

    // D: masses - heavy = dry + full headers (35 t), light = dry + the 1 t unusable header propellant
    const auto m = gnc::ship_landing_masses(cfg, ship);
    CHECK(m.heavy_kg == Approx(177000.0 + 35000.0).margin(1.0));
    CHECK(m.light_kg == Approx(177000.0 + 1000.0).margin(1.0));
    // one sea-level Raptor at sea level, thrust from the engine model; levers from the heavy CG
    const propulsion::EnginePerformance raptor(cfg.engine("raptor3"));
    const double f_min = raptor.thrust_n(raptor.config().throttle_min, p0);
    const double f_max = raptor.thrust_n(1.0, p0);
    const double f_flip = raptor.thrust_n(0.8, p0);
    const double station = std::find_if(ship.engine_groups.begin(), ship.engine_groups.end(),
                                        [](const auto& g) { return g.id == "sea_level_1"; })
                               ->station_m;
    const double lever = m.heavy_cg_station_m - station;
    const double gimbal = raptor.config().gimbal_max_rad, slew = raptor.config().gimbal_rate_rad_s;
    const double inertia = m.heavy_pitch_inertia_kg_m2;
    for (int i = 0; i < 3; ++i) {
        const double engines = i == 0 ? 3.0 : 2.0;
        const double alpha = engines * f_min * std::sin(gimbal) * lever / inertia;
        CHECK(l.rate_max_radps[static_cast<size_t>(i)] ==
              Approx(std::min(slew, std::sqrt(alpha * l.tilt_max_rad[static_cast<size_t>(i)]))).epsilon(1e-9));
    }
    // 3 -> 2 window minimum: three at minimum against the light ship's weight (0 when they cannot exceed it)
    const double out_decel = 3.0 * f_min / m.light_kg - g0;
    CHECK(l.switches[0].min_sink_mps == Approx(out_decel > 0.0 ? out_decel * raptor.config().shutdown_s : 0.0).margin(1e-12));
    CHECK(p.switch_window_from_gate);
    // catch approach: the pair runs its throttle range, (max - min) / rate
    const auto& rc = raptor.config();
    CHECK(l.approach_time_constant_s == Approx((rc.throttle_max - rc.throttle_min) / rc.throttle_rate_per_s).epsilon(1e-12));
    // ... maximum from a gate height: two at 95 % stop the heavy ship within it
    const double h = 700.0;
    CHECK(gnc::ship_switch_max_sink_mps(cfg, ship, p, h) ==
          Approx(std::sqrt(2.0 * (2.0 * f_max * 0.95 / m.heavy_kg - g0) * h)).epsilon(1e-12));
    // gate lead: the flip set at 0.8 pushes while spooling up over the Raptor's startup, the three at 95 % tilt in to
    // 20 deg at the phase's rate limit and brake
    const double brake_throttle = rc.throttle_min + (0.95 * f_max - f_min) / (f_max - f_min) * (1.0 - rc.throttle_min);
    const double f_brake = raptor.thrust_n(brake_throttle, p0);
    const gnc::FlipModel model{3.0 * f_flip / m.heavy_kg,
                               3.0 * f_flip * std::sin(gimbal) * lever / inertia,
                               rc.startup_s,
                               3.0 * f_max * 0.95 / m.heavy_kg,
                               3.0 * f_brake * std::sin(gimbal) * lever / inertia,
                               l.rate_max_radps[0],
                               20.0 * deg};
    const auto push = gnc::ship_flip_push(model);
    if (std::getenv("STARSIM_VERBOSE")) {
        std::printf("ship landing: heavy %.1f t light %.1f t, I %.4g kg m2, lever %.2f m, rates %.3f %.3f %.3f rad/s, "
                    "3->2 min sink %.2f m/s, max at %.0f m %.1f m/s, approach tau %.2f s, flip %.2f s, dv %.1f m/s, "
                    "travel %.1f m, tilt-in %.2f s %.1f m, brake from %.1f m/s %.1f m, lead %.1f m\n",
                    m.heavy_kg / 1e3, m.light_kg / 1e3, inertia, lever, l.rate_max_radps[0], l.rate_max_radps[1],
                    l.rate_max_radps[2], l.switches[0].min_sink_mps, h, gnc::ship_switch_max_sink_mps(cfg, ship, p, h),
                    l.approach_time_constant_s, push.duration_s, push.speed_mps, push.travel_m, push.tilt_in_s,
                    push.tilt_in_m, push.brake_speed_mps, push.brake_m, push.lead_m());
    }
    CHECK(p.gate_lead_m == Approx(push.lead_m()).epsilon(1e-9));
    CHECK(p.gate_lead_m > 100.0);  // the flip pushes ~100-200 m along the nose (b3 design)
    CHECK(p.gate_lead_m < 400.0);
}

TEST_CASE("Ship without landing keys is refused", "[ship][landing][config]") {
    namespace fs = std::filesystem;
    const fs::path source(STARSIM_CONFIG_DIR);
    for (const std::string key :
         {"landing_lateral_effort", "flip_throttle", "landing_engine_phases", "return_gas_kg", "entry_vent_rate_kg_s"}) {
        const auto dir = fs::temp_directory_path() / ("starsim_ship_landing_keys_" + key);
        fs::remove_all(dir);
        fs::create_directories(dir / "vehicles");
        fs::copy(source / "engines", dir / "engines", fs::copy_options::recursive);
        fs::copy_file(source / "propellants.toml", dir / "propellants.toml");
        std::ifstream in(source / "vehicles" / "starship_block3.toml");
        std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        // comment the key's line out (the phases list spans two lines: both)
        auto at = text.find("\n" + key + " =", text.find("id = \"ship\""));  // the ship's (the booster has its own phases)
        REQUIRE(at != std::string::npos);
        text.insert(at + 1, "# ");
        if (key == "landing_engine_phases") text.insert(text.find('\n', at + 2) + 1, "# ");
        std::ofstream(dir / "vehicles" / "starship_block3.toml") << text;
        try {
            config::load_vehicle(dir, "starship_block3");
            FAIL("loaded without " << key);
        } catch (const config::ConfigError& e) {
            const std::string what = e.what();
            CHECK(what.find("Schiffslandung") != std::string::npos);  // German, naming the key and the file
            CHECK(what.find(key) != std::string::npos);
            CHECK(what.find("starship_block3.toml") != std::string::npos);
        }
        fs::remove_all(dir);
    }
    // vehicles without the law load as before
    for (const char* id : {"crew_dragon", "starship_depot", "parachutist", "falcon9_block5", "starship_hls"}) {
        const auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, id);
        for (const auto& s : cfg.stages) CHECK_FALSE(s.ship_landing.has_value());
    }
}

TEST_CASE("Tanker inherits the ship landing profile", "[ship][landing][config]") {
    const auto block3 = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    const auto tanker = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_tanker");
    const auto own = gnc::ship_landing_parameters(block3, stage_of(block3, "ship"));
    const auto inherited = gnc::ship_landing_parameters(tanker, stage_of(tanker, "ship"));
    CHECK(inherited.flip_groups == own.flip_groups);
    CHECK(inherited.scvx.snapshot.phase_groups == own.scvx.snapshot.phase_groups);
    CHECK(inherited.flip_throttle == own.flip_throttle);
    CHECK(inherited.scvx.lateral_effort_fraction == own.scvx.lateral_effort_fraction);
    CHECK(inherited.scvx.snapshot.reserve_kg == own.scvx.snapshot.reserve_kg);
    CHECK(inherited.scvx.settings.limits.tilt_max_rad == own.scvx.settings.limits.tilt_max_rad);
    // the derived values follow the tanker's own dry mass
    const bool same_dry = stage_of(tanker, "ship").dry_mass_kg == stage_of(block3, "ship").dry_mass_kg;
    if (same_dry) CHECK(inherited.gate_lead_m == own.gate_lead_m);
    else CHECK(inherited.gate_lead_m != own.gate_lead_m);
    // starship_hls_launch reuses the booster and the HLS (no tower catch): no ship landing, loads as before
    const auto hls_launch = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_hls_launch");
    for (const auto& s : hls_launch.stages) CHECK_FALSE(s.ship_landing.has_value());
    CHECK(stage_of(hls_launch, "booster").return_profile.landing_engine_phases ==
          stage_of(block3, "booster").return_profile.landing_engine_phases);
}
