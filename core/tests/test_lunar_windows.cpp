// Landing window search (lunar sites plan Task 8): the listed dates have the Sun up over the stay, are ranked by the
// preferred elevation band, and every one carries the touchdown of the real planners' chain; refusals name the cause.

#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "starsim/config/lunar_site_config.hpp"
#include "starsim/config/mission_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/mission/lunar_descent.hpp"
#include "starsim/mission/lunar_site_sun.hpp"
#include "starsim/orbit/ballistic_propagator.hpp"
#include "starsim/sim/lunar_landing_mission.hpp"
#include "starsim/sim/lunar_site.hpp"
#include "starsim/sim/lunar_windows.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;

namespace {
constexpr double deg = constants::deg_to_rad;

struct Fixture {
    spice::SpiceEphemeris ephemeris;
    spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    spice::SpiceOrientation moon = spice::SpiceOrientation::moon_mean_earth();
    spice::SpiceTimeConverter time;
    sim::FlightSession prototype{ephemeris, earth, STARSIM_CONFIG_DIR, {}, &moon};
    time::Epoch utc(int year, int month, int day) const { return time.from_utc({year, month, day, 0, 0, 0.0}); }
};

struct Timed {
    sim::LunarWindowResult result;
    std::vector<double> progress;
    double seconds = 0.0;
};

Timed search(const Fixture& fx, const sim::LunarWindowRequest& request) {
    Timed t;
    const auto begin = std::chrono::steady_clock::now();
    t.result = sim::find_lunar_landing_windows(fx.prototype, STARSIM_CONFIG_DIR, request, [&](double p) {
        t.progress.push_back(p);
        return true;
    });
    t.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
    return t;
}

void print(const char* name, const Fixture& fx, const sim::LunarWindowRequest& request, const Timed& t) {
    std::printf("[lunar_windows] %s: %zu windows in %.1f s\n", name, t.result.windows.size(), t.seconds);
    for (const auto& w : t.result.windows) {
        std::printf("[lunar_windows]   start %+.2f d, touchdown %+.2f d, Sun %.2f deg az %.1f deg, daylight %.1f d, "
                    "reserve %.1f t, LLO i %.2f deg\n",
                    (w.mission_start - request.start) / 86400.0, (w.touchdown - request.start) / 86400.0,
                    w.sun_elevation_rad / deg, w.sun_azimuth_rad / deg, w.daylight_after_touchdown_s / 86400.0,
                    w.propellant_reserve_kg / 1000.0, w.llo_inclination_rad / deg);
    }
    if (t.result.refusal) std::printf("[lunar_windows]   refusal: %s\n", report::describe(*t.result.refusal).c_str());
    (void)fx;
}

double band_distance(double elevation, const config::LunarLandingMission& m) {
    return std::max({m.windows.sun_preferred_min_rad - elevation, elevation - m.windows.sun_preferred_max_rad, 0.0});
}

// The Sun above the site's horizon from touchdown through the liftoff (sampled every 30 min).
bool sun_up_over_stay(const Fixture& fx, const config::LunarSiteConfig& site, const sim::LunarLandingWindow& w) {
    for (time::Epoch t = w.touchdown; t <= w.liftoff; t += 1800.0) {
        if (mission::site_sun(fx.ephemeris, fx.moon, site.latitude_rad, site.longitude_rad, t).elevation_rad <= 0.0) {
            return false;
        }
    }
    return mission::site_sun(fx.ephemeris, fx.moon, site.latitude_rad, site.longitude_rad, w.liftoff).elevation_rad > 0.0;
}

bool site_tile(const config::LunarSiteConfig& site) {
    return sim::lunar_site_tiles_present(STARSIM_CONFIG_DIR, STARSIM_DATA_DIR, site);
}

// The outcome code a refusal names as its cause (lunar.no_window(cause)).
report::Code refusal_cause(const sim::LunarWindowResult& r) {
    REQUIRE(r.refusal);
    REQUIRE(r.refusal->code == report::Code::lunar_no_window);
    REQUIRE_FALSE(r.refusal->causes.empty());
    return r.refusal->causes.front().code;
}
}  // namespace

TEST_CASE("Apollo 11 lists 3-5 windows each with the Sun above the horizon sorted by preference",
          "[sim][lunar][lunar_windows][terrain][data]") {
    test::spice_session();
    Fixture fx;
    const auto site = config::load_lunar_site(STARSIM_CONFIG_DIR, "apollo_11");
    if (!site_tile(site)) SKIP("Apollo 11 tile not installed");
    const auto m = config::load_lunar_landing_mission(STARSIM_CONFIG_DIR, "hls_lunar_llo");
    const sim::LunarWindowRequest request{"hls_lunar_llo", site, 24.0 * 3600.0, fx.utc(2026, 11, 6)};
    const auto t = search(fx, request);
    print("Apollo 11, 24 h", fx, request, t);
    CHECK_FALSE(t.result.refusal);
    CHECK(t.result.windows.size() >= 3);
    CHECK(t.result.windows.size() <= 5);
    for (size_t i = 0; i < t.result.windows.size(); ++i) {
        const auto& w = t.result.windows[i];
        CHECK(w.mission_start >= request.start);
        CHECK(w.touchdown - request.start <= m.windows.horizon_s + 3600.0);
        CHECK(w.liftoff - w.touchdown == request.stay_s);
        CHECK(w.sun_elevation_rad > 0.0);
        CHECK(sun_up_over_stay(fx, site, w));
        CHECK(w.daylight_after_touchdown_s >= request.stay_s + sim::lunar_window_ascent_lead_s);
        CHECK(w.propellant_reserve_kg > 0.0);
        CHECK(w.orbit == mission::LunarOrbitKind::Llo);
        if (i > 0) {  // by the band distance, ties chronological
            const auto& p = t.result.windows[i - 1];
            const double dp = band_distance(p.sun_elevation_rad, m), dw = band_distance(w.sun_elevation_rad, m);
            CHECK(dp <= dw);
            if (dp == dw) CHECK(p.touchdown < w.touchdown);
        }
    }
    // progress from 0 to 1, never backwards
    REQUIRE_FALSE(t.progress.empty());
    CHECK(t.progress.front() == 0.0);
    CHECK(t.progress.back() == 1.0);
    for (size_t i = 1; i < t.progress.size(); ++i) CHECK(t.progress[i] >= t.progress[i - 1]);
}

// Ruling R4: the flight from window 1 lands in Task 9 (its start override); here every listed window is re-planned
// through the public chain - the mission started at the window's start, the plane chosen for its touchdown, the
// descent planned from the predicted LLO - and its touchdown is the listed one within 1 h.
TEST_CASE("every listed Apollo 15 window flies - the planners' chain from its start touches down within 1 h of it",
          "[sim][lunar][lunar_windows][terrain][data]") {
    test::spice_session();
    Fixture fx;
    const auto site = config::load_lunar_site(STARSIM_CONFIG_DIR, "apollo_15");
    if (!site_tile(site)) SKIP("Apollo 15 tile not installed");
    const sim::LunarWindowRequest request{"hls_lunar_llo", site, 24.0 * 3600.0, fx.utc(2026, 11, 6)};
    const auto t = search(fx, request);
    print("Apollo 15, 24 h", fx, request, t);
    REQUIRE_FALSE(t.result.windows.empty());
    auto m = config::load_lunar_landing_mission(STARSIM_CONFIG_DIR, "hls_lunar_llo");
    m.surface_stay_s = request.stay_s;
    for (const auto& w : t.result.windows) {
        m.touchdown_after_epoch_s = w.touchdown - w.mission_start;
        sim::FlightSession session{fx.ephemeris, fx.earth, STARSIM_CONFIG_DIR, {}, &fx.moon};
        const auto started = sim::start_lunar_landing_mission(session, STARSIM_CONFIG_DIR, m, w.mission_start, nullptr, &site);
        INFO(report::describe(started.refusal));
        REQUIRE(started.return_planned);
        const auto& plan = *session.lunar_outbound_plan();
        const auto& body = session.body(session.controlled_index());
        math::StateVector s{body.state().position_gcrf_m, body.state().velocity_gcrf_mps};
        s = orbit::propagate_coast(session.gravity(), s, w.mission_start, plan.tli.epoch).state;
        s.velocity_mps += plan.tli.delta_v_gcrf_mps;
        s = orbit::propagate_coast(session.gravity(), s, plan.tli.epoch, plan.insertion.epoch).state;
        s.velocity_mps += plan.insertion.delta_v_gcrf_mps;
        const time::Epoch in_llo =  // planned once settled in the LLO, as the session
            plan.insertion.epoch + sim::lunar_llo_settling_s(m, fx.ephemeris.gm_m3ps2(ephemeris::Body::Moon));
        s = orbit::propagate_coast(session.gravity(), s, plan.insertion.epoch, in_llo).state;
        auto descent = sim::lunar_landing_setup(m).descent;
        descent.planned_touchdown = plan.llo_touchdown;
        const auto& stage = body.vehicle().stages().front().config;
        const mission::LunarDescentPlanner planner(session.gravity(), session.ephemeris(), *session.moon_orientation());
        const auto d = planner.plan(s, in_llo, *session.moon_terrain(), site.latitude_rad,
                                    site.longitude_rad, stage.landing_legs.footprint_radius_m, descent);
        INFO(report::describe(d.reason));
        REQUIRE(d.feasible);
        std::printf("[lunar_windows]   re-planned: touchdown %+.3f h from the listed, crossrange %.1f km before trim\n",
                    (d.site_pass_epoch - w.touchdown) / 3600.0, d.crossrange_before_trim_m / 1000.0);
        CHECK(std::abs(d.site_pass_epoch - w.touchdown) < 3600.0);
        CHECK(std::abs(d.crossrange_before_trim_m) <= m.max_crossrange_m);
        CHECK(sun_up_over_stay(fx, site, w));
    }
}

TEST_CASE("far-side site with a too-long stay refuses with the daylight cause", "[sim][lunar][lunar_windows]") {
    test::spice_session();
    Fixture fx;
    // Tsiolkovskiy crater floor (far side, ~20.4 S 129.1 E); no tile needed: the daylight decides first
    const auto site = config::lunar_site_at(STARSIM_CONFIG_DIR, -20.4 * deg, 129.1 * deg);
    const sim::LunarWindowRequest request{"hls_lunar_llo", site, 16.0 * 86400.0, fx.utc(2026, 11, 6)};
    const auto t = search(fx, request);
    print("Tsiolkovskiy, 16 d", fx, request, t);
    CHECK(t.result.windows.empty());
    CHECK(refusal_cause(t.result) == report::Code::lunar_no_daylight_for_stay);
    CHECK(t.seconds < 30.0);  // no planner runs
}

// The far side with a day's stay (spec: one far-side site in the window search tests): windows that fly, or a refusal
// naming its cause - never an empty answer.
TEST_CASE("far-side site with a day's stay lists windows or names the cause",
          "[sim][lunar][lunar_windows][terrain][data]") {
    test::spice_session();
    Fixture fx;
    const auto site = config::lunar_site_at(STARSIM_CONFIG_DIR, -20.4 * deg, 129.1 * deg);
    if (!site_tile(site)) SKIP("Tsiolkovskiy tile not installed");
    const sim::LunarWindowRequest request{"hls_lunar_llo", site, 24.0 * 3600.0, fx.utc(2026, 11, 6)};
    const auto t = search(fx, request);
    print("Tsiolkovskiy, 24 h", fx, request, t);
    if (t.result.windows.empty()) {
        REQUIRE(t.result.refusal);
        CHECK(t.result.refusal->code == report::Code::lunar_no_window);
        CHECK_FALSE(t.result.refusal->causes.empty());
    } else {
        CHECK_FALSE(t.result.refusal);
        CHECK(t.result.windows.size() <= 5);
        for (const auto& w : t.result.windows) CHECK(sun_up_over_stay(fx, site, w));
    }
}

// The NRHO variant (spec: only where its budget closes): every candidate runs the chain with the NRHO reference; at
// 120 t the NRHO budget is short everywhere (Task 7), so the answer is the refusal naming the NRHO. ~6 min: every
// candidate builds its NRHO reference (~2 s each, 165 candidates).
TEST_CASE("NRHO mission to Apollo 11 refuses with the NRHO cause", "[sim][lunar][lunar_windows][nrho][terrain][data][nightly]") {
    test::spice_session();
    Fixture fx;
    const auto site = config::load_lunar_site(STARSIM_CONFIG_DIR, "apollo_11");
    if (!site_tile(site)) SKIP("Apollo 11 tile not installed");
    const sim::LunarWindowRequest request{"hls_lunar_nrho", site, 24.0 * 3600.0, fx.utc(2026, 11, 6)};
    const auto t = search(fx, request);
    print("Apollo 11 from the NRHO, 24 h", fx, request, t);
    if (t.result.windows.empty()) CHECK(refusal_cause(t.result) == report::Code::lunar_nrho_site_unreachable);
}

TEST_CASE("stay longer than daylight", "[sim][lunar][lunar_windows]") {
    test::spice_session();
    Fixture fx;
    const auto site = config::lunar_site_at(STARSIM_CONFIG_DIR, 0.0, 0.0);
    const sim::LunarWindowRequest request{"hls_lunar_llo", site, 15.0 * 86400.0, fx.utc(2026, 11, 6)};
    const auto t = search(fx, request);
    print("equator, 15 d", fx, request, t);
    CHECK(t.result.windows.empty());
    REQUIRE(refusal_cause(t.result) == report::Code::lunar_no_daylight_for_stay);
    // the longest daylight it names: the equatorial day, 14.77 d mean (~14.8 d measured by Task 6), shorter than 15 d
    const auto& cause = t.result.refusal->causes.front();
    REQUIRE(cause.params.size() == 2);
    std::printf("[lunar_windows]   longest daylight %.2f d\n", cause.params[1].value_si / 86400.0);
    CHECK(cause.params[1].value_si > 14.0 * 86400.0);
    CHECK(cause.params[1].value_si < 15.0 * 86400.0);
}

TEST_CASE("zero stay lists windows", "[sim][lunar][lunar_windows][terrain][data]") {
    test::spice_session();
    Fixture fx;
    const auto site = config::load_lunar_site(STARSIM_CONFIG_DIR, "apollo_11");
    if (!site_tile(site)) SKIP("Apollo 11 tile not installed");
    const sim::LunarWindowRequest request{"hls_lunar_llo", site, 0.0, fx.utc(2026, 11, 20)};
    const auto t = search(fx, request);
    print("Apollo 11, 0 h", fx, request, t);
    CHECK_FALSE(t.result.refusal);
    CHECK(t.result.windows.size() >= 3);
    for (const auto& w : t.result.windows) {
        CHECK(w.liftoff == w.touchdown);
        CHECK(w.sun_elevation_rad > 0.0);
    }
}

TEST_CASE("progress reaches 1 and cancel stops early", "[sim][lunar][lunar_windows][terrain][data]") {
    test::spice_session();
    Fixture fx;
    // reaches 1 (a refusal decided by the daylight)
    const auto equator = config::lunar_site_at(STARSIM_CONFIG_DIR, 0.0, 0.0);
    const auto refused = search(fx, {"hls_lunar_llo", equator, 15.0 * 86400.0, fx.utc(2026, 12, 1)});
    REQUIRE_FALSE(refused.progress.empty());
    CHECK(refused.progress.back() == 1.0);
    // cancel after the first planned candidate: stops there, says so, and is not cached
    const auto site = config::load_lunar_site(STARSIM_CONFIG_DIR, "apollo_11");
    if (!site_tile(site)) SKIP("Apollo 11 tile not installed");
    const sim::LunarWindowRequest request{"hls_lunar_llo", site, 24.0 * 3600.0, fx.utc(2026, 12, 1)};
    std::vector<double> seen;
    const auto cancelled = sim::find_lunar_landing_windows(fx.prototype, STARSIM_CONFIG_DIR, request, [&](double p) {
        seen.push_back(p);
        return seen.size() < 3;  // 0, the scan, the first candidate
    });
    REQUIRE(cancelled.refusal);
    CHECK(cancelled.refusal->code == report::Code::lunar_window_search_cancelled);
    CHECK(seen.size() == 3);
    CHECK(seen.back() < 1.0);
    std::vector<double> again;
    sim::find_lunar_landing_windows(fx.prototype, STARSIM_CONFIG_DIR, request, [&](double p) {
        again.push_back(p);
        return false;  // a cached result would only report 1
    });
    CHECK(again == std::vector<double>{0.0});
}
