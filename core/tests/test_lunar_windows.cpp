// Landing window search (lunar sites plan Task 8): the listed dates have the Sun up over the stay, are ranked by the
// preferred elevation band, and every one carries the touchdown of the real planners' chain; refusals name the cause.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include <catch2/catch_approx.hpp>
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
    CHECK(t.progress == std::vector<double>{0.0, 1.0});  // decided by the daylight scan, no planner runs
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
        for (const auto& w : t.result.windows) {
            CHECK(sun_up_over_stay(fx, site, w));
            CHECK(w.propellant_reserve_kg > 0.0);
        }
    }
}

// The NRHO variant (spec: only where its budget closes). At 120 t the NRHO budget is short everywhere (Task 7): ruling
// R16 decides it once per search by the budget's lower bound (one outbound plan, the site's legs at 0) - the refusal
// names the NRHO without running the candidates (488 s before the screen).
TEST_CASE("NRHO mission to Apollo 11 refuses with the NRHO cause", "[sim][lunar][lunar_windows][nrho][terrain][data]") {
    test::spice_session();
    Fixture fx;
    const auto site = config::load_lunar_site(STARSIM_CONFIG_DIR, "apollo_11");
    if (!site_tile(site)) SKIP("Apollo 11 tile not installed");
    const sim::LunarWindowRequest request{"hls_lunar_nrho", site, 24.0 * 3600.0, fx.utc(2026, 11, 6)};
    const auto t = search(fx, request);
    print("Apollo 11 from the NRHO, 24 h", fx, request, t);
    CHECK(t.result.windows.empty());
    REQUIRE(refusal_cause(t.result) == report::Code::lunar_nrho_site_unreachable);
    const auto& cause = t.result.refusal->causes.front();
    REQUIRE(cause.params.size() == 2);
    CHECK(cause.params[1].value_si > 0.0);  // the lower bound's shortfall
    // decided before the candidates: 0, the scan, 1
    CHECK(t.progress == std::vector<double>{0.0, 0.05, 1.0});
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

// Task 8 review I1: the plane-through-site transfer lets the arrival drift (-10 .. +21 h); a candidate whose arrival
// comes after its planned pass is re-solved (the start earlier by the drift, the plane epoch kept) and listed - before
// the fix it was dropped as "no daylight" (Apollo 11 candidate +9.96 d took the pass at +14.0 d).
TEST_CASE("a candidate whose arrival drifts past its pass is re-solved, not dropped",
          "[sim][lunar][lunar_windows][terrain][data]") {
    test::spice_session();
    Fixture fx;
    const auto site = config::load_lunar_site(STARSIM_CONFIG_DIR, "apollo_11");
    if (!site_tile(site)) SKIP("Apollo 11 tile not installed");
    const auto m = config::load_lunar_landing_mission(STARSIM_CONFIG_DIR, "hls_lunar_llo");
    const sim::LunarWindowRequest request{"hls_lunar_llo", site, 24.0 * 3600.0, fx.utc(2026, 11, 6)};
    const auto t = search(fx, request);
    print("Apollo 11, 24 h (drift)", fx, request, t);
    REQUIRE_FALSE(t.result.refusal);
    const double nominal_s =
        m.flight_time_s + sim::lunar_first_pass_after_arrival_s(m, fx.ephemeris.gm_m3ps2(ephemeris::Body::Moon));
    int resolved = 0;
    for (const auto& w : t.result.windows) {
        // started earlier than transfer + lead by the drift (hours), not a pass days later
        const double extra_s = (w.touchdown - w.mission_start) - nominal_s;
        std::printf("[lunar_windows]   touchdown %+.2f d: started %.2f h earlier than transfer + lead\n",
                    (w.touchdown - request.start) / 86400.0, extra_s / 3600.0);
        if (extra_s > 3600.0 && extra_s < 86400.0) ++resolved;
        CHECK(w.mission_start >= request.start);
        CHECK(sun_up_over_stay(fx, site, w));
    }
    CHECK(resolved >= 1);
}

// Task 8 re-review N1 (ruling R18): a candidate right after `earliest` cannot start earlier, so a positive arrival drift
// misses its planned pass. Either the later pass is re-run as the planned touchdown and listed with that run's own
// touchdown and reserve (Sun over the stay holds), or the candidate is refused as lunar.arrival_after_site_pass with the
// real lateness (N2: not 0 h).
TEST_CASE("a candidate that cannot start earlier is re-run at its later pass or refused as late",
          "[sim][lunar][lunar_windows][terrain][data]") {
    test::spice_session();
    Fixture fx;
    const auto site = config::load_lunar_site(STARSIM_CONFIG_DIR, "apollo_11");
    if (!site_tile(site)) SKIP("Apollo 11 tile not installed");
    const auto m = config::load_lunar_landing_mission(STARSIM_CONFIG_DIR, "hls_lunar_llo");
    const auto env = sim::lunar_window_environment(fx.prototype);
    const double gm = fx.ephemeris.gm_m3ps2(ephemeris::Body::Moon);
    const double lead_s = m.flight_time_s + sim::lunar_first_pass_after_arrival_s(m, gm);
    int listed_later = 0, refused_late = 0;
    for (const double stay_s : {24.0 * 3600.0, 2.0 * 3600.0})
    for (int k = 0; k < 24; ++k) {
        const sim::LunarWindowRequest request{"hls_lunar_llo", site, stay_s, fx.utc(2026, 11, 11) + k * 3.0 * 3600.0};
        const time::Epoch td = request.start + lead_s;  // the earliest candidate: its start cannot move earlier
        const auto r = sim::evaluate_lunar_window_candidate(env, STARSIM_CONFIG_DIR, request, td);
        if (r.window) {
            const auto& w = *r.window;
            std::printf("[lunar_windows]   start 11-11 + %d h: listed, touchdown %+.3f d from it, reserve %.2f t\n", 3 * k,
                        (w.touchdown - td) / 86400.0, w.propellant_reserve_kg / 1000.0);
            CHECK(w.mission_start >= request.start);
            CHECK(sun_up_over_stay(fx, site, w));
            CHECK(w.propellant_reserve_kg > 0.0);
            if (w.touchdown - td > 6.0 * 86400.0) ++listed_later;  // the later pass, a whole node period on
        } else {
            std::printf("[lunar_windows]   start 11-11 + %d h: %s\n", 3 * k, report::describe(r.cause).c_str());
            if (r.cause.code == report::Code::lunar_arrival_after_site_pass) {
                ++refused_late;
                REQUIRE_FALSE(r.cause.params.empty());
                CHECK(r.cause.params.front().value_si > 3600.0);  // the real lateness, not a clamped 0
            }
        }
    }
    // the 24 h stay does not fit the daylight left at the later pass, the 2 h stay does
    CHECK(refused_late >= 1);
    CHECK(listed_later >= 1);
}

// Review I2: a refusal from missing data is not cached - after the download the same request searches again.
TEST_CASE("a refusal from a missing site tile is not cached", "[sim][lunar][lunar_windows]") {
    test::spice_session();
    Fixture fx;
    const auto site = config::lunar_site_at(STARSIM_CONFIG_DIR, 10.0 * deg, -30.0 * deg);  // Oceanus Procellarum
    if (site_tile(site)) SKIP("the tile of 10 N 30 W is installed");
    const sim::LunarWindowRequest request{"hls_lunar_llo", site, 24.0 * 3600.0, fx.utc(2026, 11, 6)};
    const auto first = search(fx, request);
    print("10 N 30 W without its tile", fx, request, first);
    REQUIRE(first.result.windows.empty());
    CHECK(report::id_of(refusal_cause(first.result)).starts_with("data."));
    const auto second = search(fx, request);
    REQUIRE(second.progress.size() > 1);  // a cached result would only report 1
    CHECK(second.progress.front() == 0.0);
}

// Review I3: the search takes its sources and flags at the call; the prototype changing under it (the sim thread)
// changes neither the result nor the cache key.
TEST_CASE("the search reads the prototype only at the call", "[sim][lunar][lunar_windows][terrain][data]") {
    test::spice_session();
    Fixture fx;
    const auto site = config::load_lunar_site(STARSIM_CONFIG_DIR, "apollo_11");
    if (!site_tile(site)) SKIP("Apollo 11 tile not installed");
    const sim::LunarWindowRequest request{"hls_lunar_llo", site, 24.0 * 3600.0, fx.utc(2026, 11, 20)};
    const bool detail = fx.prototype.moon_surface_detail(), rocks = fx.prototype.moon_rocks();
    sim::clear_lunar_window_cache();
    const auto changed = sim::find_lunar_landing_windows(fx.prototype, STARSIM_CONFIG_DIR, request, [&](double) {
        fx.prototype.set_moon_surface_detail(!detail);  // as the sim thread could
        fx.prototype.set_moon_rocks(!rocks);
        return true;
    });
    fx.prototype.set_moon_surface_detail(detail);
    fx.prototype.set_moon_rocks(rocks);
    std::vector<double> hit;  // cached under the flags at the call
    sim::find_lunar_landing_windows(fx.prototype, STARSIM_CONFIG_DIR, request, [&](double p) {
        hit.push_back(p);
        return true;
    });
    CHECK(hit == std::vector<double>{1.0});
    sim::clear_lunar_window_cache();
    const auto fresh = sim::find_lunar_landing_windows(fx.prototype, STARSIM_CONFIG_DIR, request, {});
    REQUIRE(changed.windows.size() == fresh.windows.size());
    CHECK_FALSE(changed.windows.empty());
    for (size_t i = 0; i < fresh.windows.size(); ++i) {
        CHECK(changed.windows[i].mission_start == fresh.windows[i].mission_start);
        CHECK(changed.windows[i].touchdown == fresh.windows[i].touchdown);
        CHECK(changed.windows[i].propellant_reserve_kg == fresh.windows[i].propellant_reserve_kg);
    }
}

// Review I4 / ruling R15: the stay's plane change - the site's distance from the landing plane after the stay, turned
// back as 2 v_llo sin(di / 2) - against the site turned rigidly about the Moon's pole at the sidereal rate. Per day of
// stay for the planes the outbound plan chooses (no terrain). To first order the site leaves the plane at
// Omega sqrt(sin^2 i - sin^2 phi); where the plane puts the site at its track's vertex (sin i ~ |sin phi|) the motion
// is second order and the first-order rate says ~0.
TEST_CASE("the stay's plane change follows the Moon's rotation under the landing plane", "[sim][lunar][lunar_windows]") {
    test::spice_session();
    Fixture fx;
    auto m = config::load_lunar_landing_mission(STARSIM_CONFIG_DIR, "hls_lunar_llo");
    const double gm = fx.ephemeris.gm_m3ps2(ephemeris::Body::Moon);
    m.touchdown_after_epoch_s = m.flight_time_s + sim::lunar_first_pass_after_arrival_s(m, gm);
    const double r_llo = constants::moon_mean_radius_m + m.llo_altitude_m;
    const double v_llo = std::sqrt(gm / r_llo);
    struct Site {
        const char* name;
        config::LunarSiteConfig config;
    };
    const std::vector<Site> sites{{"Apollo 11", config::load_lunar_site(STARSIM_CONFIG_DIR, "apollo_11")},
                                  {"Apollo 15", config::load_lunar_site(STARSIM_CONFIG_DIR, "apollo_15")},
                                  {"equator 0 N 0 E", config::lunar_site_at(STARSIM_CONFIG_DIR, 0.0, 0.0)},
                                  {"Mons Mouton", config::load_lunar_site(STARSIM_CONFIG_DIR, "mons_mouton_im2")}};
    const time::Epoch start = fx.utc(2026, 11, 13);
    for (const auto& s : sites) {
        sim::FlightSession session{fx.ephemeris, fx.earth, STARSIM_CONFIG_DIR, {}, &fx.moon};
        const auto state = sim::lunar_parking_state(session, m, start, m.parking_inclination_rad);
        REQUIRE(state);
        session.place_in_orbit(m.vehicle_id, *state, start);
        REQUIRE(session.plan_lunar_outbound(
            sim::lunar_outbound_parameters(m, session.gravity(), session.ephemeris(), start, &s.config)));
        const auto& plan = *session.lunar_outbound_plan();
        REQUIRE(plan.llo_plane_normal_moon_inertial);
        REQUIRE(plan.llo_touchdown);
        const auto& n = *plan.llo_plane_normal_moon_inertial;
        const double day_mps =
            sim::lunar_stay_plane_change_mps(fx.moon, s.config, n, *plan.llo_touchdown, 86400.0, r_llo, gm);
        const auto rotation = fx.moon.fixed_to_gcrf(*plan.llo_touchdown).rotation;
        const math::Vec3 pole = rotation * math::Vec3{0.0, 0.0, 1.0};
        const double i = std::acos(std::clamp(dot(n, pole), -1.0, 1.0));
        const double lat = s.config.latitude_rad, lon = s.config.longitude_rad;
        const math::Vec3 s0 =
            rotation * math::Vec3{std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat)};
        // reference: the site turned rigidly about the pole at touchdown by the sidereal rate (Rodrigues; no libration)
        const double turn = constants::two_pi / constants::sidereal_month_s * 86400.0;
        const math::Vec3 s1 =
            s0 * std::cos(turn) + cross(pole, s0) * std::sin(turn) + pole * (dot(pole, s0) * (1.0 - std::cos(turn)));
        const double reference_mps = 2.0 * v_llo * std::sin(0.5 * std::asin(std::min(std::abs(dot(n, s1)), 1.0)));
        // first order (meaningless where the site is the track's northern / southern vertex, sin i ~ |sin phi|)
        const double sin_phi = std::sin(lat);
        const double rate = constants::two_pi / constants::sidereal_month_s *
                            std::sqrt(std::max(std::sin(i) * std::sin(i) - sin_phi * sin_phi, 0.0));
        std::printf("[lunar_windows] stay plane change %s: i %.1f deg, %.1f m/s per day (rigid rotation %.1f, first "
                    "order %.1f), 3 d %.0f m/s, 7 d %.0f m/s\n",
                    s.name, i / deg, day_mps, reference_mps, 2.0 * v_llo * std::sin(0.5 * rate * 86400.0),
                    sim::lunar_stay_plane_change_mps(fx.moon, s.config, n, *plan.llo_touchdown, 3 * 86400.0, r_llo, gm),
                    sim::lunar_stay_plane_change_mps(fx.moon, s.config, n, *plan.llo_touchdown, 7 * 86400.0, r_llo, gm));
        CHECK(day_mps == Catch::Approx(reference_mps).epsilon(0.03).margin(1.0));
        // the bound without a plane is never below it; no stay, no turn
        CHECK(sim::lunar_stay_plane_change_mps(fx.moon, s.config, std::nullopt, *plan.llo_touchdown, 86400.0, r_llo,
                                               gm) >= day_mps);
        CHECK(sim::lunar_stay_plane_change_mps(fx.moon, s.config, n, *plan.llo_touchdown, 0.0, r_llo, gm) == 0.0);
    }
}

// Ruling R16: the NRHO screen's [windows] nrho_outbound_floor_mps is the least outbound the planner flies - checked over
// two synodic months of starts every 6 h (the 9:2 NRHO's phasing repeats then; the outbound is site-independent).
// ~14 min. STARSIM_SWEEP_DAYS (default 60), STARSIM_SWEEP_STEP_H (default 6).
TEST_CASE("NRHO outbound never below the mission's floor", "[sim][lunar][lunar_windows][nrho][nightly]") {
    test::spice_session();
    Fixture fx;
    const auto m = config::load_lunar_landing_mission(STARSIM_CONFIG_DIR, "hls_lunar_nrho");
    REQUIRE(m.windows.nrho_outbound_floor_mps);
    const auto site = config::load_lunar_site(STARSIM_CONFIG_DIR, "apollo_11");
    const char* days_env = std::getenv("STARSIM_SWEEP_DAYS");
    const char* step_env = std::getenv("STARSIM_SWEEP_STEP_H");
    const double days = days_env ? std::atof(days_env) : 60.0;
    const double step_s = (step_env ? std::atof(step_env) : 6.0) * 3600.0;
    const time::Epoch first = fx.utc(2026, 11, 6);
    for (double dt = 0.0; dt <= days * 86400.0; dt += step_s) {
        const time::Epoch start = first + dt;
        sim::FlightSession session{fx.ephemeris, fx.earth, STARSIM_CONFIG_DIR, {}, &fx.moon};
        const auto state = sim::lunar_parking_state(session, m, start, m.parking_inclination_rad);
        if (!state) {
            std::printf("[nrho_sweep] %+.2f d: no parking plane\n", dt / 86400.0);
            continue;
        }
        session.place_in_orbit(m.vehicle_id, *state, start);
        const bool planned = session.plan_lunar_outbound(
            sim::lunar_outbound_parameters(m, session.gravity(), session.ephemeris(), start, &site));
        if (!planned) {
            std::printf("[nrho_sweep] %+.2f d: refused %s\n", dt / 86400.0,
                        report::describe(session.lunar_status()).c_str());
            continue;
        }
        const auto& plan = *session.lunar_outbound_plan();
        const double tli = norm(plan.tli.delta_v_gcrf_mps);
        const double flyby = plan.powered_flyby ? norm(plan.powered_flyby->delta_v_gcrf_mps) : 0.0;
        const double insertion = norm(plan.insertion.delta_v_gcrf_mps);
        std::printf("[nrho_sweep] %+.2f d: tli %.0f swing_by %.0f insertion %.0f sum %.0f m/s\n", dt / 86400.0, tli,
                    flyby, insertion, tli + flyby + insertion);
        std::fflush(stdout);
        CHECK(tli + flyby + insertion >= *m.windows.nrho_outbound_floor_mps);
    }
}

// R15 on the shipped mission (Mons Mouton, 24 h): the start budgets the stay's plane change and still closes.
TEST_CASE("the shipped LLO mission budgets the stay's plane change", "[sim][lunar][lunar_windows][terrain][data]") {
    test::spice_session();
    Fixture fx;
    const auto m = config::load_lunar_landing_mission(STARSIM_CONFIG_DIR, "hls_lunar_llo");
    const auto site = config::load_lunar_site(STARSIM_CONFIG_DIR, m.site_id);
    if (!site_tile(site)) SKIP("Mons Mouton tile not installed");
    const time::Epoch epoch = fx.time.from_utc(m.epoch_utc);
    const auto start_with_stay = [&](double stay_s) {
        auto mission = m;
        mission.surface_stay_s = stay_s;
        sim::FlightSession session{fx.ephemeris, fx.earth, STARSIM_CONFIG_DIR, {}, &fx.moon};
        return sim::start_lunar_landing_mission(session, STARSIM_CONFIG_DIR, mission, epoch);
    };
    const auto none = start_with_stay(0.0);
    const auto day = start_with_stay(m.surface_stay_s);
    INFO(report::describe(day.refusal));
    REQUIRE(none.return_planned);
    REQUIRE(day.return_planned);
    REQUIRE(day.lunar_stay_plane_change_mps);
    std::printf("[lunar_windows] Mons Mouton %.0f h: stay plane change %.1f m/s, margin %.2f t (no stay %.2f t)\n",
                m.surface_stay_s / 3600.0, *day.lunar_stay_plane_change_mps, *day.lunar_budget_margin_kg / 1000.0,
                *none.lunar_budget_margin_kg / 1000.0);
    CHECK(*none.lunar_stay_plane_change_mps == 0.0);
    CHECK(*day.lunar_stay_plane_change_mps > 0.0);
    CHECK(*day.lunar_budget_margin_kg < *none.lunar_budget_margin_kg);
    CHECK(*day.lunar_budget_margin_kg > 0.0);
}

// Ruling R4 (moved from Task 8): the mission started through the start options at window 1 plans the descent for the
// listed touchdown.
TEST_CASE("every listed Apollo 15 window flies: start the mission at window 1 and the descent plan's touchdown is within 1 h of the listed one",
          "[sim][lunar][lunar_windows][terrain][data]") {
    test::spice_session();
    Fixture fx;
    const auto site = config::load_lunar_site(STARSIM_CONFIG_DIR, "apollo_15");
    if (!site_tile(site)) SKIP("Apollo 15 tile not installed");
    const sim::LunarWindowRequest request{"hls_lunar_llo", site, 24.0 * 3600.0, fx.utc(2026, 11, 6)};
    const auto t = search(fx, request);
    REQUIRE_FALSE(t.result.windows.empty());
    const auto& w = t.result.windows.front();
    sim::FlightSession session{fx.ephemeris, fx.earth, STARSIM_CONFIG_DIR, {}, &fx.moon};
    sim::StartOptions options;
    options.lunar = sim::LunarStartOverride{site, request.stay_s, w.mission_start, w.touchdown};
    const auto started = sim::start_mission(session, STARSIM_CONFIG_DIR, "hls_lunar_llo", {}, fx.time, options);
    INFO(report::describe(started.refusal));
    REQUIRE(started.return_planned);
    CHECK(started.epoch == w.mission_start);
    // fly until the descent is planned in the LLO
    for (time::Epoch e = w.mission_start; !session.lunar_descent_plan() && e < w.touchdown; e += 1800.0) {
        session.advance_to(e, 200'000'000);
    }
    const auto* plan = session.lunar_descent_plan();
    REQUIRE(plan);
    INFO(report::describe(plan->reason));
    REQUIRE(plan->feasible);
    std::printf("[lunar_windows]   start options: descent touchdown %+.3f h from the listed\n",
                (plan->site_pass_epoch - w.touchdown) / 3600.0);
    CHECK(std::abs(plan->site_pass_epoch - w.touchdown) < 3600.0);
}
