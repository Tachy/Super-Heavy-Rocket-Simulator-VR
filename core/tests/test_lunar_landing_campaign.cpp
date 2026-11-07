// Lunar landing campaign (spec l4): from PDI with state +-1 km / +-5 m/s per axis, thrust +-2 %, propellant +-5 % and
// touchdown points on the real terrain (the admissible point nearest to a random spot within 1 km of the site): 21 / 21
// land within 1 m at <= 1 m/s. Aborts from 3 / 1 / 0.3 km reach orbit. Single cases: "[.lunar_case]" with
// STARSIM_LUNAR_CASE=<seed>. Flown on the shipped composed surface (Moon procedural surface plan, Task 5 fix round 1):
// LOLA + procedural craters and roughness + rocks under the footpads, the touchdown points chosen with the hazard check
// (as FlightSession::plan_lunar_landing does).

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <random>

#include <catch2/catch_test_macros.hpp>

#include "starsim/config/lunar_site_config.hpp"
#include "starsim/config/mission_config.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/mission/lunar_descent.hpp"
#include "starsim/orbit/ballistic_propagator.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/sim/lunar_landing_mission.hpp"
#include "starsim/sim/lunar_windows.hpp"
#include "starsim/sim/lunar_site.hpp"
#include "starsim/sim/terrain_cache.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "lunar_test_support.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {
constexpr double deg = constants::deg_to_rad;
constexpr double approach_south_rad = 180.0 * deg;  // polar pass heading south over the site (Mons Mouton)
constexpr double footprint_radius_m = 12.0;          // HLS legs (l2)

struct Dispersion {
    Vec3 position_m;               // added to the PDI state (GCRF), each axis +-1 km
    Vec3 velocity_mps;             // +-5 m/s per axis
    double thrust_factor = 1.0;    // +-2 %
    double mass_factor = 1.0;      // propellant +-5 %
    double east_m = 0.0, north_m = 0.0;  // touchdown target spot (within 1 km of the site)
};

Dispersion dispersion(unsigned seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> unit(-1.0, 1.0);
    Dispersion d;
    d.position_m = {1000.0 * unit(rng), 1000.0 * unit(rng), 1000.0 * unit(rng)};
    d.velocity_mps = {5.0 * unit(rng), 5.0 * unit(rng), 5.0 * unit(rng)};
    d.thrust_factor = 1.0 + 0.02 * unit(rng);
    d.mass_factor = 1.0 + 0.05 * unit(rng);
    const double r = 1000.0 * std::sqrt(0.5 * (unit(rng) + 1.0)), a = constants::pi * unit(rng);
    d.east_m = r * std::cos(a);
    d.north_m = r * std::sin(a);
    return d;
}

struct Fixture {
    bool spice = (test::spice_session(), true);
    spice::SpiceEphemeris ephemeris;
    spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    spice::SpiceOrientation moon_me = spice::SpiceOrientation::moon_mean_earth();
    sim::FlightSession session{ephemeris, earth, STARSIM_CONFIG_DIR, {}, &moon_me};
    time::Epoch t0 = spice::SpiceTimeConverter().from_utc({2026, 11, 6, 0, 0, 0.0});
    double approach_azimuth_rad = approach_south_rad;
    std::shared_ptr<const environment::DetailedLunarTerrain> surface;
    sim::LunarSite site;
    Fixture() : Fixture(config::load_lunar_site(STARSIM_CONFIG_DIR, "mons_mouton_im2")) {}
    explicit Fixture(const config::LunarSiteConfig& config) {
        surface = sim::shared_moon_surface(STARSIM_CONFIG_DIR, STARSIM_DATA_DIR, {true, true}, config);
        session.set_moon_terrain(surface);
        site = sim::lunar_site(config, surface->lola());
        session.set_lunar_landing_site(site);
    }
    // The admissible touchdown point nearest to the spot (east, north) metres from the site centre.
    mission::TouchdownPoint point_near(double east_m, double north_m) const {
        const double r = constants::moon_mean_radius_m;
        const double lat = site.config.latitude_rad + north_m / r;
        const double lon = site.config.longitude_rad + east_m / (r * std::cos(site.config.latitude_rad));
        const auto p = mission::select_touchdown_point(
            *surface, lat, lon, approach_azimuth_rad, footprint_radius_m, mission::LunarDescentParameters{},
            mission::touchdown_hazard_limits(config::load_vehicle(STARSIM_CONFIG_DIR, "starship_hls").stages.front()));
        REQUIRE(p);
        return *p;
    }
    double mass_kg() const { return session.body(0).vehicle().mass_properties().mass_kg; }
    double altitude_m() const {
        const auto c = ephemeris.state_gcrf(ephemeris::Body::Moon, session.epoch());
        return norm(session.body(0).state().position_gcrf_m - c.position_m) - constants::moon_mean_radius_m;
    }
};

struct Run {
    sim::LegTouchdown outcome = sim::LegTouchdown::None;
    double offset_m = 0.0, sink_mps = 0.0, m0_t = 0.0, used_t = 0.0, slope_deg = 0.0;
    double first_contact_offset_m = 0.0, lateral_mps = 0.0;  // at the first leg contact (review M-c)
    report::Report status;
};

// PDI at the real mass (566 t, propellant x mass factor) with the braking range the planner computes for that mass,
// the dispersion on the state and the engines, the touchdown point near the spot; flies to the leg judgement.
// On a site's planned track (approach azimuth other than due south) the PDI state lies on that ground track.
Run fly(Fixture& fx, const Dispersion& d) {
    const auto point = fx.point_near(d.east_m, d.north_m);
    const double propellant = test::hls_pdi_propellant_kg * d.mass_factor;
    const double range = test::pdi_braking_range_m(fx.ephemeris, point, 120e3 + propellant);
    if (fx.approach_azimuth_rad == approach_south_rad) {
        test::place_hls_at_pdi(fx.session, fx.ephemeris, fx.moon_me, point, fx.t0, propellant, 0.0, range, d.position_m,
                               d.velocity_mps);
    } else {
        test::place_hls_at_pdi_on_track(fx.session, fx.ephemeris, fx.moon_me, point, fx.t0, propellant, range,
                                        fx.approach_azimuth_rad, d.position_m, d.velocity_mps);
    }
    fx.session.set_thrust_dispersion(0, d.thrust_factor);
    if (std::getenv("STARSIM_CAMPAIGN_TRACE")) {
        std::printf("[trace] point %.6f %.6f deg, height %.1f m (surface %.1f, lola %.1f), offset %.0f m, PDI %.1f t\n",
                    point.latitude_rad / deg, point.longitude_rad / deg, point.height_m,
                    fx.surface->height_m(point.latitude_rad, point.longitude_rad),
                    fx.surface->lola().height_m(point.latitude_rad, point.longitude_rad), point.offset_m,
                    fx.mass_kg() / 1000.0);
    }
    Run r;
    r.m0_t = fx.mass_kg() / 1000.0;
    r.slope_deg = point.slope_rad / deg;
    REQUIRE(fx.session.begin_powered_descent(point, fx.approach_azimuth_rad));
    // 0.1 s steps (as test_lunar_powered_descent.cpp)
    for (int i = 0; i < 1200 * 10 && fx.session.lunar_step() == sim::LunarStep::PoweredDescent; ++i) {
        fx.session.advance_to(fx.session.epoch() + 0.1, 100);
        if (std::getenv("STARSIM_CAMPAIGN_TRACE") && (i % 10 == 0 || fx.session.lunar_touchdown())) {  // diagnostics: the last 500 m once a second, every tick after the first contact
            const auto& b = fx.session.body(0);
            const auto g = b.ground_clearance(fx.session.environment(), fx.session.epoch());
            if (g.height_m < 500.0) {
                const auto c = fx.ephemeris.state_gcrf(ephemeris::Body::Moon, fx.session.epoch());
                const Vec3 fixed = transpose(fx.moon_me.fixed_to_gcrf(fx.session.epoch()).rotation) *
                                   (b.state().position_gcrf_m - c.position_m);
                const double lat = std::asin(fixed.z / norm(fixed)), lon = std::atan2(fixed.y, fixed.x);
                std::printf("[trace] t %.1f clearance %.2f m legs %.2f mass %.1f t  cg %.6f %.6f deg h %.1f m ground %.1f "
                            "m  %s\n", i * 0.1, g.height_m, b.vehicle().leg_deployment(), fx.mass_kg() / 1000.0, lat / deg,
                            lon / deg, norm(fixed) - constants::moon_mean_radius_m, fx.surface->height_m(lat, lon),
                            report::describe(fx.session.lunar_status()).c_str());
            }
        }
    }
    if (std::getenv("STARSIM_CAMPAIGN_TRACE")) {  // diagnostics: the settling after the descent, every 0.1 s
        for (int i = 0; i < 50; ++i) {
            fx.session.advance_to(fx.session.epoch() + 0.1, 100);
            const auto& b = fx.session.body(0);
            const auto c = fx.ephemeris.state_gcrf(ephemeris::Body::Moon, fx.session.epoch());
            const Vec3 r = b.state().position_gcrf_m - c.position_m;
            const double tilt = std::acos(std::clamp(dot(b.state().attitude.rotate(Vec3{1.0, 0.0, 0.0}), math::normalized(r)), -1.0, 1.0));
            std::printf("[trace] settle %.1f s: tilt %.2f deg, mass %.2f t, offset %.2f m, %s\n", 0.1 * (i + 1), tilt / deg,
                        fx.mass_kg() / 1000.0, fx.session.lunar_touchdown() ? fx.session.lunar_touchdown()->offset_m : -1.0,
                        report::describe(fx.session.lunar_status()).c_str());
        }
    } else {
        fx.session.advance_to(fx.session.epoch() + 5.0, 2000);
    }
    r.status = fx.session.lunar_status();
    r.used_t = r.m0_t - fx.mass_kg() / 1000.0;
    if (const auto& td = fx.session.lunar_touchdown()) {
        r.outcome = td->outcome;
        r.offset_m = td->offset_m;
        r.sink_mps = td->sink_mps;
        r.first_contact_offset_m = td->first_contact_offset_m;
        r.lateral_mps = td->lateral_mps;
    }
    return r;
}

Run fly(const Dispersion& d) {
    Fixture fx;
    return fly(fx, d);
}

// A site's first listed window (24 h stay, search from 2026-11-06) and the geometry the planners chose for it: the
// mission started at the window's start, TLI and insertion flown impulsively, the descent planned once settled in the
// LLO (as test_lunar_windows "every listed Apollo 15 window flies"). `refusal` when the search lists no window.
struct SiteWindow {
    config::LunarSiteConfig site;
    std::optional<sim::LunarLandingWindow> window;
    std::optional<report::Report> refusal;
    time::Epoch pdi_epoch;
    double approach_azimuth_rad = 0.0;
};

SiteWindow first_window(const config::LunarSiteConfig& site) {
    test::spice_session();
    spice::SpiceEphemeris ephemeris;
    auto earth = spice::SpiceOrientation::earth();
    auto moon = spice::SpiceOrientation::moon_mean_earth();
    SiteWindow out{site};
    const sim::LunarWindowRequest request{"hls_lunar_llo", site, 24.0 * 3600.0,
                                          spice::SpiceTimeConverter().from_utc({2026, 11, 6, 0, 0, 0.0})};
    sim::FlightSession prototype{ephemeris, earth, STARSIM_CONFIG_DIR, {}, &moon};
    const auto found = sim::find_lunar_landing_windows(prototype, STARSIM_CONFIG_DIR, request, [](double) { return true; });
    if (found.windows.empty()) {
        out.refusal = found.refusal;
        std::printf("[lunar_campaign] %s: no window, %s\n", site.id.c_str(),
                    found.refusal ? report::describe(*found.refusal).c_str() : "(no refusal)");
        return out;
    }
    const auto& w = found.windows.front();
    out.window = w;
    auto m = config::load_lunar_landing_mission(STARSIM_CONFIG_DIR, "hls_lunar_llo");
    m.surface_stay_s = request.stay_s;
    m.touchdown_after_epoch_s = w.touchdown - w.mission_start;
    sim::FlightSession session{ephemeris, earth, STARSIM_CONFIG_DIR, {}, &moon};
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
    const time::Epoch in_llo = plan.insertion.epoch + sim::lunar_llo_settling_s(m, ephemeris.gm_m3ps2(ephemeris::Body::Moon));
    s = orbit::propagate_coast(session.gravity(), s, plan.insertion.epoch, in_llo).state;
    auto descent = sim::lunar_landing_setup(m).descent;
    descent.planned_touchdown = plan.llo_touchdown;
    const auto& stage = body.vehicle().stages().front().config;
    const mission::LunarDescentPlanner planner(session.gravity(), session.ephemeris(), *session.moon_orientation());
    const auto d = planner.plan(s, in_llo, *session.moon_terrain(), site.latitude_rad, site.longitude_rad,
                                stage.landing_legs.footprint_radius_m, descent);
    INFO(report::describe(d.reason));
    REQUIRE(d.feasible);
    out.pdi_epoch = d.pdi_epoch;
    out.approach_azimuth_rad = d.approach_azimuth_rad;
    std::printf("[lunar_campaign] %s: window 1 touchdown +%.2f d after 2026-11-06, Sun %.1f deg, reserve %.1f t, LLO i "
                "%.1f deg; descent plan: pass %+.3f h from the listed, approach azimuth %.1f deg\n",
                site.id.c_str(), (w.touchdown - request.start) / 86400.0, w.sun_elevation_rad / deg,
                w.propellant_reserve_kg / 1000.0, w.llo_inclination_rad / deg, (d.site_pass_epoch - w.touchdown) / 3600.0,
                d.approach_azimuth_rad / deg);
    std::fflush(stdout);
    return out;
}

// The site's campaign case `seed`: the dispersed PDI (as the Mons Mouton campaign) on the window's planned track at its
// planned PDI epoch, on the site's composed surface (its tile).
Run fly_at(const SiteWindow& sw, const Dispersion& d) {
    Fixture fx(sw.site);
    fx.t0 = sw.pdi_epoch;
    fx.approach_azimuth_rad = sw.approach_azimuth_rad;
    return fly(fx, d);
}

int fly_seeds(unsigned first, unsigned last, const SiteWindow* sw = nullptr) {
    int landed = 0;
    for (unsigned seed = first; seed <= last; ++seed) {
        const auto d = dispersion(seed);
        const auto run = sw ? fly_at(*sw, d) : fly(d);
        if (sw) std::printf("[lunar_campaign] %s ", sw->site.id.c_str());
        std::printf("[lunar_campaign] case %2u: thrust %.3f, propellant %.3f, spot %.0f/%.0f m, slope %.1f deg: %s, offset %.2f m "
                    "(%.2f m at first contact, lateral %.3f m/s), sink %.2f m/s, %.1f t at PDI, %.1f t used, %s\n",
                    seed, d.thrust_factor, d.mass_factor, d.east_m, d.north_m, run.slope_deg, sim::id_of(run.outcome),
                    run.offset_m, run.first_contact_offset_m, run.lateral_mps, run.sink_mps, run.m0_t, run.used_t, report::describe(run.status).c_str());
        std::fflush(stdout);
        if (run.outcome == sim::LegTouchdown::Landed && run.offset_m < 1.0 && run.sink_mps <= 1.1) ++landed;
    }
    std::printf("[lunar_campaign] seeds %u-%u: landed %d of %u\n", first, last, landed, last - first + 1);
    return landed;
}

// The site campaign (lunar sites plan Task 13): seeds `first`..`last` at the site's first listed window land within the
// gates of the Mons Mouton campaign - or, when `refusal_cause` is set, the window search lists none and names it.
void site_campaign(const config::LunarSiteConfig& site, unsigned first, unsigned last,
                   std::optional<report::Code> refusal_cause = std::nullopt) {
    if (!sim::lunar_site_tiles_present(STARSIM_CONFIG_DIR, STARSIM_DATA_DIR, site)) SKIP(site.id + " tile not installed");
    const auto sw = first_window(site);
    if (refusal_cause) {
        REQUIRE_FALSE(sw.window);
        REQUIRE(sw.refusal);
        CHECK(sw.refusal->code == report::Code::lunar_no_window);
        REQUIRE_FALSE(sw.refusal->causes.empty());
        CHECK(sw.refusal->causes.front().code == *refusal_cause);
        return;
    }
    if (sw.refusal) UNSCOPED_INFO(report::describe(*sw.refusal));
    REQUIRE(sw.window);
    CHECK(fly_seeds(first, last, &sw) == static_cast<int>(last - first + 1));
}

config::LunarSiteConfig tsiolkovskiy() { return config::lunar_site_at(STARSIM_CONFIG_DIR, -20.4 * deg, 129.1 * deg); }
config::LunarSiteConfig high_north() { return config::lunar_site_at(STARSIM_CONFIG_DIR, 75.0 * deg, 30.0 * deg); }
}  // namespace

// Free sites at their first listed window, one seed each (lunar sites plan Task 13).
TEST_CASE("Site campaign: Apollo 11 at window 1 lands within 1 m",
          "[sim][lunar_campaign][validation][terrain][data][slow]") {
    site_campaign(config::load_lunar_site(STARSIM_CONFIG_DIR, "apollo_11"), 1, 1);
}

TEST_CASE("Site campaign: Apollo 15 at window 1 lands within 1 m",
          "[sim][lunar_campaign][validation][terrain][data][slow]") {
    site_campaign(config::load_lunar_site(STARSIM_CONFIG_DIR, "apollo_15"), 1, 1);
}

TEST_CASE("Site campaign: Apollo 17 at window 1 lands within 1 m",
          "[sim][lunar_campaign][validation][terrain][data][slow]") {
    site_campaign(config::load_lunar_site(STARSIM_CONFIG_DIR, "apollo_17"), 1, 1);
}

TEST_CASE("Site campaign: Tsiolkovskiy (far side) at window 1 lands within 1 m",
          "[sim][lunar_campaign][validation][terrain][data][slow]") {
    site_campaign(tsiolkovskiy(), 1, 1);
}

TEST_CASE("Site campaign: 75 N 30 E at window 1 lands within 1 m",
          "[sim][lunar_campaign][validation][terrain][data][slow]") {
    site_campaign(high_north(), 1, 1);
}

// Nightly: 7 seeds per Apollo site.
TEST_CASE("Site campaign nightly: Apollo 11, seeds 1-7", "[sim][lunar_campaign][validation][terrain][data][nightly]") {
    site_campaign(config::load_lunar_site(STARSIM_CONFIG_DIR, "apollo_11"), 1, 7);
}

TEST_CASE("Site campaign nightly: Apollo 15, seeds 1-7", "[sim][lunar_campaign][validation][terrain][data][nightly]") {
    site_campaign(config::load_lunar_site(STARSIM_CONFIG_DIR, "apollo_15"), 1, 7);
}

TEST_CASE("Site campaign nightly: Apollo 17, seeds 1-7", "[sim][lunar_campaign][validation][terrain][data][nightly]") {
    site_campaign(config::load_lunar_site(STARSIM_CONFIG_DIR, "apollo_17"), 1, 7);
}

// Split into three cases so ctest runs them in parallel; together 21 / 21 (spec).
TEST_CASE("Lunar landing campaign: dispersed descents 1-7 land within 1 m",
          "[sim][lunar_campaign][validation][terrain][data][slow]") {
    CHECK(fly_seeds(1, 7) == 7);
}

TEST_CASE("Lunar landing campaign: dispersed descents 8-14 land within 1 m",
          "[sim][lunar_campaign][validation][terrain][data][slow]") {
    CHECK(fly_seeds(8, 14) == 7);
}

TEST_CASE("Lunar landing campaign: dispersed descents 15-21 land within 1 m",
          "[sim][lunar_campaign][validation][terrain][data][slow]") {
    CHECK(fly_seeds(15, 21) == 7);
}

TEST_CASE("Lunar landing campaign: a single case", "[.lunar_case]") {
    const char* seed = std::getenv("STARSIM_LUNAR_CASE");
    const unsigned s = seed ? static_cast<unsigned>(std::atoi(seed)) : 1u;
    if (const char* id = std::getenv("STARSIM_LUNAR_SITE")) {  // a preset site at its first window (site campaign)
        const auto sw = first_window(config::load_lunar_site(STARSIM_CONFIG_DIR, id));
        if (sw.window) fly_seeds(s, s, &sw);
        return;
    }
    fly_seeds(s, s);
}

TEST_CASE("Aborts from 3, 1 and 0.3 km reach orbit", "[sim][lunar_campaign][abort][validation][terrain][data][slow]") {
    for (const double h : {3000.0, 1000.0, 300.0}) {
        Fixture fx;
        const auto point = fx.point_near(0.0, 0.0);
        const double range = test::pdi_braking_range_m(fx.ephemeris, point, 120e3 + test::hls_pdi_propellant_kg);
        test::place_hls_at_pdi(fx.session, fx.ephemeris, fx.moon_me, point, fx.t0, test::hls_pdi_propellant_kg, 0.0,
                               range);
        REQUIRE(fx.session.begin_powered_descent(point, fx.approach_azimuth_rad));
        REQUIRE(test::descend_to(fx.session, fx.ephemeris, h, point));
        const double m0 = fx.mass_kg();
        REQUIRE(fx.session.abort_landing(0));
        fx.session.advance_to(fx.session.epoch() + 3.0 * 3600.0, 50'000'000);
        std::printf("[lunar_campaign] abort from %.1f km at %.1f t: %.1f t used, altitude %.1f km, %s\n", h / 1000.0,
                    m0 / 1000.0, (m0 - fx.mass_kg()) / 1000.0, fx.altitude_m() / 1000.0,
                    report::describe(fx.session.lunar_status()).c_str());
        std::fflush(stdout);
        CAPTURE(h);
        CHECK(fx.session.lunar_step() == sim::LunarStep::InLlo);
        CHECK(fx.altitude_m() > 90e3);
    }
}
