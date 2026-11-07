#include "starsim/sim/lunar_windows.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

#include "flight_session_lunar_detail.hpp"
#include "starsim/config/mission_config.hpp"
#include "starsim/gnc/lunar_ascent_guidance.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/mission/lunar_budget.hpp"
#include "starsim/mission/lunar_descent.hpp"
#include "starsim/mission/lunar_site_sun.hpp"
#include "starsim/orbit/ballistic_propagator.hpp"
#include "starsim/sim/lunar_landing_mission.hpp"
#include "starsim/sim/terrain_cache.hpp"

namespace starsim::sim {

namespace {

// [E] share of the progress for the daylight scan and the ranking (the chain runs take the rest)
constexpr double scan_progress = 0.05;

bool debug_enabled() { return std::getenv("STARSIM_DEBUG_LUNAR_WINDOWS") != nullptr; }

// ---- cache (per process; the result depends only on the key's inputs and the config / data) ----
std::mutex& cache_mutex() {
    static std::mutex m;
    return m;
}
std::map<std::string, LunarWindowResult>& cache() {
    static std::map<std::string, LunarWindowResult> c;
    return c;
}

std::string cache_key(const LunarWindowEnvironment& env, const std::filesystem::path& config_dir,
                      const LunarWindowRequest& r) {
    char buffer[256];
    std::snprintf(buffer, sizeof buffer, "|%.6f|%.6f|%lld|%lld|%d%d", r.site.latitude_rad, r.site.longitude_rad,
                  static_cast<long long>(std::llround(r.stay_s)),
                  static_cast<long long>(std::llround(r.start.tdb_seconds_past_j2000() / 3600.0)),
                  env.moon_surface_detail ? 1 : 0, env.moon_rocks ? 1 : 0);
    return config_dir.string() + "|" + r.mission_id + "|" + r.site.id + buffer;
}

// A refusal from missing data or the environment (a file not downloaded, no Moon orientation, an internal error): not
// a property of the site and the dates, so not cached.
bool environmental(const report::Report& r) {
    if (r.code == report::Code::internal_error || r.code == report::Code::lunar_landing_no_moon_orientation ||
        report::id_of(r.code).starts_with("data.")) {
        return true;
    }
    return std::any_of(r.causes.begin(), r.causes.end(), [](const report::Report& c) { return environmental(c); });
}

// ---- refusal causes ----
// The cause a refused candidate counts under: the first named one in the report tree (depth first, the outer first -
// lunar.nrho_site_unreachable before its lunar.budget_short), else the innermost first cause.
report::Report candidate_cause(const report::Report& refusal) {
    using report::Code;
    const auto named = [](Code c) {
        return c == Code::lunar_no_daylight_for_stay || c == Code::lunar_nrho_site_unreachable ||
               c == Code::lunar_budget_short || c == Code::lunar_no_site_pass ||
               c == Code::lunar_arrival_after_site_pass;
    };
    std::vector<const report::Report*> stack{&refusal};
    while (!stack.empty()) {
        const auto* r = stack.back();
        stack.pop_back();
        if (named(r->code)) return *r;
        for (auto it = r->causes.rbegin(); it != r->causes.rend(); ++it) stack.push_back(&*it);
    }
    const report::Report* leaf = &refusal;
    while (!leaf->causes.empty()) leaf = &leaf->causes.front();
    return *leaf;
}

struct CauseTally {
    struct Entry {
        int count = 0;
        size_t first = 0;  // order of the first occurrence (ties: the earlier, i.e. the better-ranked candidate)
        report::Report report;
    };
    std::map<report::Code, Entry> entries;
    void add(const report::Report& cause) {
        auto [it, inserted] = entries.try_emplace(cause.code);
        if (inserted) {
            it->second.first = entries.size();
            it->second.report = cause;
        }
        ++it->second.count;
    }
    std::optional<report::Report> most_frequent() const {
        const Entry* best = nullptr;
        for (const auto& [code, e] : entries) {
            if (!best || e.count > best->count || (e.count == best->count && e.first < best->first)) best = &e;
        }
        if (!best) return std::nullopt;
        return best->report;
    }
};

report::Report no_window(const report::Report& cause) {
    return report::with_cause(report::make(report::Code::lunar_no_window), cause);
}

report::Report no_daylight(double stay_s, double daylight_s) {
    return report::make(report::Code::lunar_no_daylight_for_stay,
                        {report::time("stay", stay_s), report::time("daylight", std::max(daylight_s, 0.0))});
}

// ---- candidates ----
struct Candidate {
    time::Epoch touchdown;
    mission::DaylightInterval daylight;
    double band_distance_rad = 0.0;
};

double band_distance(double elevation_rad, const config::LunarLandingMission::Windows& w) {
    if (elevation_rad < w.sun_preferred_min_rad) return w.sun_preferred_min_rad - elevation_rad;
    if (elevation_rad > w.sun_preferred_max_rad) return elevation_rad - w.sun_preferred_max_rad;
    return 0.0;
}

// A session of its own, as the flight's: the snapshot's sources, grids and fidelity flags.
std::unique_ptr<FlightSession> session_from(const LunarWindowEnvironment& env,
                                            const std::filesystem::path& config_dir) {
    const auto& s = env.sources;
    auto session = std::make_unique<FlightSession>(*s.ephemeris, *s.earth, config_dir, s.gravity, s.moon,
                                                   s.ephemeris_node_spacing_s > 0.0);
    session->set_moon_surface_detail(env.moon_surface_detail);
    session->set_moon_rocks(env.moon_rocks);
    session->set_boil_off(true);  // as start_mission
    return session;
}

double llo_period_s(const config::LunarLandingMission& m, double gm_moon) {
    const double r = constants::moon_mean_radius_m + m.llo_altitude_m;
    return constants::two_pi * std::sqrt(r * r * r / gm_moon);
}

// ---- the chain for one candidate ----
struct ChainResult {
    std::optional<LunarLandingWindow> window;
    report::Report cause;  // when no window
};

class Chain {
public:
    Chain(LunarWindowEnvironment env, std::filesystem::path config_dir, config::LunarLandingMission m,
          config::LunarSiteConfig site, time::Epoch earliest_start, double lead_s)
        : env_(std::move(env)), config_dir_(std::move(config_dir)), m_(std::move(m)), site_(std::move(site)),
          earliest_start_(earliest_start), lead_s_(lead_s) {
        const double gm_moon = env_.sources.ephemeris->gm_m3ps2(ephemeris::Body::Moon);
        settling_s_ = lunar_llo_settling_s(m_, gm_moon);
        period_s_ = llo_period_s(m_, gm_moon);
    }

    ChainResult run(const Candidate& c) const {
        try {
            return run_impl(c);
        } catch (const std::exception& e) {  // a planner's integrator, the ephemeris range
            return {std::nullopt, detail::exception_report(e)};
        }
    }

private:
    ChainResult run_impl(const Candidate& c) const {
        const bool nrho = m_.lunar_orbit == "nrho";
        double after_s = m_.flight_time_s + lead_s_;  // the LLO plane holds the site at the candidate then
        for (int resolve = 0;; ++resolve) {
            auto m = m_;
            m.touchdown_after_epoch_s = after_s;
            const time::Epoch start = c.touchdown - after_s;
            const auto session = session_from(env_, config_dir_);
            const auto started = start_lunar_landing_mission(*session, config_dir_, m, start, nullptr, &site_);
            if (!started.refusal.ok()) return {std::nullopt, started.refusal};
            if (!started.lunar_planned_touchdown || !started.lunar_budget_margin_kg) {
                return {std::nullopt, report::make(report::Code::internal_error)};
            }
            if (nrho) return finish(c, m, start, *started.lunar_planned_touchdown, *started.lunar_budget_margin_kg,
                                    mission::LloTarget{}.inclination_rad, false);
            const auto descent = descent_plan(*session, m, start);
            if (!descent.feasible) return {std::nullopt, descent.reason};
            const time::Epoch touchdown = descent.site_pass_epoch;
            const auto* plan = session->lunar_outbound_plan();
            const double drift_s = (plan->insertion.epoch - start) - m.flight_time_s;  // arrival vs start + transfer
            if (debug_enabled()) {
                std::fprintf(stderr,
                             "[lunar_windows]   resolve %d: start %+.2f h, arrival drift %+.2f h, pass %+.3f d, "
                             "crossrange %.1f km\n",
                             resolve, -after_s / 3600.0, drift_s / 3600.0, (touchdown - c.touchdown) / 86400.0,
                             descent.crossrange_before_trim_m / 1000.0);
            }
            // the descent took a later pass than the one the plane was chosen for: the arrival came too late for it
            const bool missed = touchdown - c.touchdown > period_s_;
            if (missed) {
                const double next_after_s = m.flight_time_s + lead_s_ + std::max(drift_s, 0.0);
                if (resolve < lunar_window_max_resolves && next_after_s > after_s + 1.0 &&
                    c.touchdown - next_after_s >= earliest_start_) {
                    after_s = next_after_s;  // start earlier by the drift, the plane epoch kept
                    continue;
                }
                return later_pass(c, m, start, touchdown);
            }
            // the stay's plane change from the touchdown the descent flies (the start budgeted the planned pass)
            const double margin_kg = lunar_llo_budget_margin_kg(*session, m, start, touchdown);
            return finish(c, m, start, touchdown, margin_kg, plane_inclination(*session, *plan, m, touchdown), false);
        }
    }

    // The planned pass is missed and the start cannot move earlier: the flight that takes the later pass has its plane
    // chosen for that pass (ruling R18). One more run of the chain with it as the planned touchdown, same start; listed
    // only when that run holds (its own touchdown, Sun and reserve), else the candidate is refused as late.
    ChainResult later_pass(const Candidate& c, const config::LunarLandingMission& m0, time::Epoch start,
                           time::Epoch pass) const {
        const auto late = [&](time::Epoch taken) {
            return ChainResult{std::nullopt, report::make(report::Code::lunar_arrival_after_site_pass,
                                                          {report::time("late", std::max(taken - c.touchdown, 0.0))})};
        };
        auto m = m0;
        m.touchdown_after_epoch_s = pass - start;
        const auto session = session_from(env_, config_dir_);
        const auto started = start_lunar_landing_mission(*session, config_dir_, m, start, nullptr, &site_);
        if (!started.refusal.ok() || !started.lunar_planned_touchdown) return late(pass);
        const auto descent = descent_plan(*session, m, start);
        if (!descent.feasible) return late(pass);
        const time::Epoch touchdown = descent.site_pass_epoch;
        if (touchdown - pass > period_s_) return late(pass);  // not the pass the plane was chosen for
        const auto* plan = session->lunar_outbound_plan();
        const double margin_kg = lunar_llo_budget_margin_kg(*session, m, start, touchdown);
        auto result = finish(c, m, start, touchdown, margin_kg, plane_inclination(*session, *plan, m, touchdown), true);
        if (debug_enabled()) {
            std::fprintf(stderr, "[lunar_windows]   later pass %+.3f d re-run: pass %+.3f d, reserve %.2f t, %s\n",
                         (pass - c.touchdown) / 86400.0, (touchdown - c.touchdown) / 86400.0, margin_kg / 1000.0,
                         result.window ? "listed" : "refused");
        }
        if (!result.window) return late(pass);
        return result;
    }

    static double plane_inclination(const FlightSession& session, const mission::OutboundPlan& plan,
                                    const config::LunarLandingMission& m, time::Epoch touchdown) {
        if (plan.llo_plane_normal_moon_inertial) {
            const math::Vec3 pole = session.moon_orientation()->fixed_to_gcrf(touchdown).rotation *
                                    math::Vec3{0.0, 0.0, 1.0};
            return std::acos(std::clamp(dot(*plan.llo_plane_normal_moon_inertial, pole), -1.0, 1.0));
        }
        if (m.llo_inclination_rad) return *m.llo_inclination_rad;
        return mission::LloTarget{}.inclination_rad;
    }

    // The hard Sun rule on the touchdown the flight takes (daylight from it through the stay and the ascent), then the
    // window.
    ChainResult finish(const Candidate& c, const config::LunarLandingMission& m, time::Epoch start,
                       time::Epoch touchdown, double margin_kg, double inclination_rad, bool missed) const {
        const auto& day = c.daylight;
        if (touchdown < day.sunrise || touchdown + m.surface_stay_s + lunar_window_ascent_lead_s > day.sunset) {
            if (debug_enabled()) {
                std::fprintf(stderr, "[lunar_windows]   touchdown %+.3f d from the candidate, daylight %.3f .. %.3f d%s\n",
                             (touchdown - c.touchdown) / 86400.0, (day.sunrise - c.touchdown) / 86400.0,
                             (day.sunset - c.touchdown) / 86400.0, missed ? " (planned pass missed)" : "");
            }
            return {std::nullopt, no_daylight(m.surface_stay_s, day.sunset - std::max(touchdown, day.sunrise))};
        }
        const auto sun = mission::site_sun(*env_.sources.ephemeris, *env_.sources.moon, site_.latitude_rad,
                                           site_.longitude_rad, touchdown);
        LunarLandingWindow w;
        w.mission_start = start;
        w.touchdown = touchdown;
        w.liftoff = touchdown + m.surface_stay_s;
        w.sun_elevation_rad = sun.elevation_rad;
        w.sun_azimuth_rad = sun.azimuth_rad;
        w.daylight_after_touchdown_s = day.sunset - touchdown;
        w.propellant_reserve_kg = margin_kg;
        w.llo_inclination_rad = inclination_rad;
        w.orbit = m.lunar_orbit == "nrho" ? mission::LunarOrbitKind::Nrho : mission::LunarOrbitKind::Llo;
        return {w, {}};
    }

    // The descent plan the flight makes in the LLO (FlightSession::plan_lunar_landing's planner and surface), from the
    // LLO the outbound plan predicts: TLI and insertion impulsive at their nodes in the full force model, planned when
    // the session is settled in the LLO (lunar_llo_settling_s after the insertion; a later arrival skips early passes).
    mission::LunarDescentPlan descent_plan(const FlightSession& session, const config::LunarLandingMission& m,
                                           time::Epoch start) const {
        const auto& plan = *session.lunar_outbound_plan();
        const auto& body = session.body(session.controlled_index());
        math::StateVector s{body.state().position_gcrf_m, body.state().velocity_gcrf_mps};
        const auto& gravity = session.gravity();
        s = orbit::propagate_coast(gravity, s, start, plan.tli.epoch).state;
        s.velocity_mps += plan.tli.delta_v_gcrf_mps;
        s = orbit::propagate_coast(gravity, s, plan.tli.epoch, plan.insertion.epoch).state;
        s.velocity_mps += plan.insertion.delta_v_gcrf_mps;
        const time::Epoch in_llo = plan.insertion.epoch + settling_s_;  // the session plans once settled in the LLO
        s = orbit::propagate_coast(gravity, s, plan.insertion.epoch, in_llo).state;
        auto descent = lunar_landing_setup(m).descent;
        descent.planned_touchdown = plan.llo_touchdown ? plan.llo_touchdown : lunar_planned_touchdown(m, start);
        const auto& stage = body.vehicle().stages().front().config;
        const double footprint_m = stage.landing_legs.footprint_radius_m;
        const mission::LunarDescentPlanner planner(gravity, session.ephemeris(), *session.moon_orientation());
        const auto* surface =
            dynamic_cast<const environment::DetailedLunarTerrain*>(session.moon_surface_shared().get());
        if (surface && surface->detail_on() && stage.landing_legs.foot_pad_radius_m > 0.0) {
            return planner.plan(s, in_llo, *surface, site_.latitude_rad, site_.longitude_rad,
                                footprint_m, descent, mission::touchdown_hazard_limits(stage));
        }
        return planner.plan(s, in_llo, *session.moon_terrain(), site_.latitude_rad, site_.longitude_rad,
                            footprint_m, descent);
    }

    LunarWindowEnvironment env_;
    std::filesystem::path config_dir_;
    config::LunarLandingMission m_;
    config::LunarSiteConfig site_;
    time::Epoch earliest_start_;
    double lead_s_ = 0.0;
    double settling_s_ = 0.0, period_s_ = 0.0;
};

// ---- NRHO screen (ruling R16) ----
// A lower bound of every candidate's budget, decided once: the vehicle as placed at `start`, the outbound at the
// mission's [windows] nrho_outbound_floor_mps (TLI, swing-by and insertion ignitions kept), the legs after the arrival
// nominal ([budget]), the site-dependent legs (nrho_plane_change, stay_plane_change) at 0. The refusal when even that
// does not close; empty when it closes, the mission has no floor or no parking plane at `start` (the candidates decide).
std::optional<report::Report> nrho_screen(const LunarWindowEnvironment& env, const std::filesystem::path& config_dir,
                                          const config::LunarLandingMission& m, const config::LunarSiteConfig& site,
                                          time::Epoch start) {
    if (!m.windows.nrho_outbound_floor_mps) return std::nullopt;
    const auto session = session_from(env, config_dir);
    const auto state = lunar_parking_state(*session, m, start, m.parking_inclination_rad);
    if (!state) return std::nullopt;
    session->place_in_orbit(m.vehicle_id, *state, start);
    mission::OutboundPlan floor;
    floor.tli.delta_v_gcrf_mps = {*m.windows.nrho_outbound_floor_mps, 0.0, 0.0};
    floor.powered_flyby = gnc::BurnNode{start, {0.0, 0.0, 0.0}};  // its ignition; the delta-v is in the floor
    floor.insertion.delta_v_gcrf_mps = {0.0, 0.0, 0.0};
    const auto& v = session->body(session->controlled_index()).vehicle();
    const auto& sets = lunar_landing_setup(m).guidance.braking_sets;
    const auto legs = lunar_mission_legs(m, floor, gnc::groups_vacuum_isp_s(v, sets.front()),
                                         gnc::groups_vacuum_isp_s(v, sets.back()), 0.0, 0.0);
    const double mass_kg = v.mass_properties().mass_kg;
    const auto budget = mission::lunar_budget(
        mass_kg, mass_kg - v.propellant_kg(),
        m.reserve_kg + session->settling_gas_bound_kg() * static_cast<double>(lunar_in_orbit_ignitions(legs)), legs);
    if (debug_enabled()) {
        std::fprintf(stderr, "[lunar_windows] NRHO screen: outbound floor %.0f m/s, lower-bound margin %.2f t\n",
                     *m.windows.nrho_outbound_floor_mps, budget.margin_kg / 1000.0);
    }
    if (budget.closes) return std::nullopt;
    const auto unreachable = report::make(report::Code::lunar_nrho_site_unreachable,
                                          {report::angle("latitude", site.latitude_rad),
                                           report::mass("budget_short", -budget.margin_kg)});
    return report::with_cause(unreachable, budget.reason);
}

struct SearchOutcome {
    LunarWindowResult result;
    bool cacheable = false;  // complete, and no refusal from missing data or the environment
};

SearchOutcome search(const LunarWindowEnvironment& env, const std::filesystem::path& config_dir,
                     const LunarWindowRequest& request, const std::function<bool(double)>& progress) {
    bool cancelled = false;
    const auto report_progress = [&](double p) {
        if (progress && !progress(std::clamp(p, 0.0, 1.0))) cancelled = true;
        return !cancelled;
    };
    SearchOutcome out;
    auto& result = out.result;
    const auto cancel = [&] {
        result.refusal = report::make(report::Code::lunar_window_search_cancelled);
        out.cacheable = false;
        return out;
    };
    const auto refuse = [&](const report::Report& cause, bool cacheable) {
        result.refusal = no_window(cause);
        out.cacheable = cacheable;
        report_progress(1.0);
        return out;
    };
    if (!report_progress(0.0)) return cancel();
    const auto& sources = env.sources;
    if (!sources.moon) return refuse(report::make(report::Code::lunar_landing_no_moon_orientation), false);
    auto m = config::load_lunar_landing_mission(config_dir, request.mission_id);
    m.surface_stay_s = std::max(request.stay_s, 0.0);
    const auto& windows = m.windows;
    const double gm_moon = sources.ephemeris->gm_m3ps2(ephemeris::Body::Moon);
    const double lead_s = lunar_first_pass_after_arrival_s(m, gm_moon);  // arrival -> the planned pass
    const double hold_s = m.surface_stay_s + lunar_window_ascent_lead_s;
    const time::Epoch earliest = request.start + m.flight_time_s + lead_s;
    const time::Epoch last = request.start + windows.horizon_s;
    const double lat = request.site.latitude_rad, lon = request.site.longitude_rad;

    // 1-2: daylight over the horizon and the stay after it; candidates inside the intervals that hold the stay
    const auto days = mission::daylight_intervals(*sources.ephemeris, *sources.moon, lat, lon, request.start,
                                                  windows.horizon_s + hold_s + lunar_window_candidate_step_s);
    double longest_s = 0.0;  // the longest daylight a touchdown in [earliest, last] could use
    std::vector<Candidate> candidates;
    for (const auto& day : days) {
        if (day.sunset > earliest && day.sunrise <= last) {
            longest_s = std::max(longest_s, day.sunset - std::max(day.sunrise, earliest));
        }
        for (time::Epoch t = day.sunrise + 0.5 * lunar_window_candidate_step_s;
             t <= last && t + hold_s <= day.sunset; t += lunar_window_candidate_step_s) {
            if (t < earliest) continue;
            const auto sun = mission::site_sun(*sources.ephemeris, *sources.moon, lat, lon, t);
            candidates.push_back({t, day, band_distance(sun.elevation_rad, windows)});
        }
    }
    if (candidates.empty()) return refuse(no_daylight(m.surface_stay_s, longest_s), true);
    // 3: by preference, ties chronological
    std::stable_sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
        return a.band_distance_rad < b.band_distance_rad;
    });
    if (!report_progress(scan_progress)) return cancel();

    // NRHO (ruling R16): decided once by the budget's lower bound, from the best-ranked candidate's start
    if (m.lunar_orbit == "nrho") {
        std::optional<report::Report> screened;
        try {
            screened = nrho_screen(env, config_dir, m, request.site,
                                   candidates.front().touchdown - m.flight_time_s - lead_s);
        } catch (const std::exception& e) {
            return refuse(detail::exception_report(e), false);
        }
        if (screened) return refuse(*screened, true);
        if (!report_progress(scan_progress)) return cancel();
    }

    // the site's surface held for the whole search (the sessions share it); its tile missing refuses every candidate
    std::shared_ptr<const environment::DetailedLunarTerrain> surface;
    try {
        surface = shared_moon_surface(config_dir, config_dir.parent_path() / "data", {true, true}, request.site);
    } catch (const std::exception& e) {
        return refuse(detail::exception_report(e), false);
    }

    const Chain chain(env, config_dir, m, request.site, request.start, lead_s);
    // 4: the real chain, best first, until `count` pass. Spread over the lunar days first (a list of dates, not five
    // slots of one morning): a candidate whose daylight already holds a window waits for a second round.
    CauseTally causes;
    bool from_environment = false;
    const size_t wanted = static_cast<size_t>(windows.count);
    const bool debug = debug_enabled();
    std::vector<time::Epoch> days_with_window;  // by sunrise
    std::vector<size_t> deferred;
    size_t tried = 0;
    const auto began = std::chrono::steady_clock::now();
    const auto attempt = [&](size_t i) {
        const auto outcome = chain.run(candidates[i]);
        if (outcome.window) {
            result.windows.push_back(*outcome.window);
            days_with_window.push_back(candidates[i].daylight.sunrise);
        } else {
            causes.add(candidate_cause(outcome.cause));
            from_environment = from_environment || environmental(outcome.cause);
        }
        ++tried;
        if (debug) {
            std::fprintf(stderr, "[lunar_windows] %.1f s: candidate %zu/%zu td %+.2f d band %.2f deg: %s\n",
                         std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count(), i + 1,
                         candidates.size(), (candidates[i].touchdown - request.start) / 86400.0,
                         candidates[i].band_distance_rad / constants::deg_to_rad,
                         outcome.window ? "window" : report::describe(outcome.cause).c_str());
        }
        const double done = std::max(static_cast<double>(result.windows.size()) / static_cast<double>(wanted),
                                     static_cast<double>(tried) / static_cast<double>(candidates.size()));
        return report_progress(scan_progress + (1.0 - scan_progress) * done);
    };
    for (size_t i = 0; i < candidates.size() && result.windows.size() < wanted; ++i) {
        const auto sunrise = candidates[i].daylight.sunrise;
        if (std::find(days_with_window.begin(), days_with_window.end(), sunrise) != days_with_window.end()) {
            deferred.push_back(i);
            continue;
        }
        if (!attempt(i)) return cancel();
    }
    for (size_t k = 0; k < deferred.size() && result.windows.size() < wanted; ++k) {
        if (!attempt(deferred[k])) return cancel();
    }
    // listed by preference on their own touchdown's Sun (the pass may differ from the candidate by up to half a
    // revolution), ties chronological
    std::stable_sort(result.windows.begin(), result.windows.end(),
                     [&](const LunarLandingWindow& a, const LunarLandingWindow& b) {
                         const double da = band_distance(a.sun_elevation_rad, windows);
                         const double db = band_distance(b.sun_elevation_rad, windows);
                         if (da != db) return da < db;
                         return a.touchdown < b.touchdown;
                     });
    if (result.windows.empty()) {
        const auto cause = causes.most_frequent();
        result.refusal = no_window(cause ? *cause : report::make(report::Code::internal_error));
    }
    out.cacheable = !from_environment;
    report_progress(1.0);
    return out;
}

}  // namespace

LunarWindowEnvironment lunar_window_environment(const FlightSession& session) {
    return {session.environment_sources(), session.moon_surface_detail(), session.moon_rocks()};
}

LunarWindowResult find_lunar_landing_windows(const LunarWindowEnvironment& environment,
                                             const std::filesystem::path& config_dir, const LunarWindowRequest& request,
                                             const std::function<bool(double)>& progress) {
    const auto key = cache_key(environment, config_dir, request);
    {
        const std::lock_guard lock(cache_mutex());
        const auto it = cache().find(key);
        // a hit from a start up to the rounding earlier may list a mission start before this request's: search again
        if (it != cache().end() &&
            std::none_of(it->second.windows.begin(), it->second.windows.end(),
                         [&](const LunarLandingWindow& w) { return w.mission_start < request.start; })) {
            if (progress) progress(1.0);
            return it->second;
        }
    }
    auto outcome = search(environment, config_dir, request, progress);
    if (outcome.cacheable) {
        const std::lock_guard lock(cache_mutex());
        cache().insert_or_assign(key, outcome.result);
    }
    return std::move(outcome.result);
}

LunarWindowResult find_lunar_landing_windows(const FlightSession& prototype, const std::filesystem::path& config_dir,
                                             const LunarWindowRequest& request,
                                             const std::function<bool(double)>& progress) {
    return find_lunar_landing_windows(lunar_window_environment(prototype), config_dir, request, progress);
}

LunarCandidateResult evaluate_lunar_window_candidate(const LunarWindowEnvironment& environment,
                                                     const std::filesystem::path& config_dir,
                                                     const LunarWindowRequest& request, time::Epoch touchdown) {
    auto m = config::load_lunar_landing_mission(config_dir, request.mission_id);
    m.surface_stay_s = std::max(request.stay_s, 0.0);
    const auto& sources = environment.sources;
    const double gm_moon = sources.ephemeris->gm_m3ps2(ephemeris::Body::Moon);
    const double lead_s = lunar_first_pass_after_arrival_s(m, gm_moon);
    const double hold_s = m.surface_stay_s + lunar_window_ascent_lead_s;
    const double lat = request.site.latitude_rad, lon = request.site.longitude_rad;
    const auto days = mission::daylight_intervals(*sources.ephemeris, *sources.moon, lat, lon, request.start,
                                                  m.windows.horizon_s + hold_s + lunar_window_candidate_step_s);
    for (const auto& day : days) {
        if (touchdown < day.sunrise || touchdown + hold_s > day.sunset) continue;
        const auto sun = mission::site_sun(*sources.ephemeris, *sources.moon, lat, lon, touchdown);
        const Candidate c{touchdown, day, band_distance(sun.elevation_rad, m.windows)};
        const Chain chain(environment, config_dir, m, request.site, request.start, lead_s);
        auto out = chain.run(c);
        return {out.window, out.cause};
    }
    return {std::nullopt, no_daylight(m.surface_stay_s, 0.0)};
}

void clear_lunar_window_cache() {
    const std::lock_guard lock(cache_mutex());
    cache().clear();
}

}  // namespace starsim::sim
