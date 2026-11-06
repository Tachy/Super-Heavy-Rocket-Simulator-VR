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
#include "starsim/math/constants.hpp"
#include "starsim/mission/lunar_descent.hpp"
#include "starsim/mission/lunar_site_sun.hpp"
#include "starsim/orbit/ballistic_propagator.hpp"
#include "starsim/sim/lunar_landing_mission.hpp"
#include "starsim/sim/terrain_cache.hpp"

namespace starsim::sim {

namespace {

// [E] share of the progress for the daylight scan and the ranking (the chain runs take the rest)
constexpr double scan_progress = 0.05;

// ---- cache (per process; the result depends only on the key's inputs and the config) ----
std::mutex& cache_mutex() {
    static std::mutex m;
    return m;
}
std::map<std::string, LunarWindowResult>& cache() {
    static std::map<std::string, LunarWindowResult> c;
    return c;
}

std::string cache_key(const FlightSession& prototype, const std::filesystem::path& config_dir,
                      const LunarWindowRequest& r) {
    char buffer[256];
    std::snprintf(buffer, sizeof buffer, "|%.6f|%.6f|%lld|%lld|%d%d", r.site.latitude_rad, r.site.longitude_rad,
                  static_cast<long long>(std::llround(r.stay_s)),
                  static_cast<long long>(std::llround(r.start.tdb_seconds_past_j2000() / 3600.0)),
                  prototype.moon_surface_detail() ? 1 : 0, prototype.moon_rocks() ? 1 : 0);
    return config_dir.string() + "|" + r.mission_id + "|" + r.site.id + buffer;
}

// ---- refusal causes ----
// The cause a refused candidate counts under: the first named one in the report tree (depth first, the outer first -
// lunar.nrho_site_unreachable before its lunar.budget_short), else the innermost first cause.
report::Report candidate_cause(const report::Report& refusal) {
    using report::Code;
    const auto named = [](Code c) {
        return c == Code::lunar_no_daylight_for_stay || c == Code::lunar_nrho_site_unreachable ||
               c == Code::lunar_budget_short || c == Code::lunar_no_site_pass;
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

// ---- the chain for one candidate ----
struct ChainResult {
    std::optional<LunarLandingWindow> window;
    report::Report cause;  // when no window
};

class Chain {
public:
    Chain(const FlightSession& prototype, const std::filesystem::path& config_dir, const config::LunarLandingMission& m,
          const config::LunarSiteConfig& site, double settling_s)
        : prototype_(prototype), config_dir_(config_dir), m_(m), site_(site),
          sources_(prototype.environment_sources()), settling_s_(settling_s) {}

    ChainResult run(const Candidate& c, double lead_s) const {
        try {
            return run_impl(c, lead_s);
        } catch (const std::exception& e) {  // a planner's integrator, the ephemeris range
            return {std::nullopt, detail::exception_report(e)};
        }
    }

private:
    ChainResult run_impl(const Candidate& c, double lead_s) const {
        auto m = m_;
        m.touchdown_after_epoch_s = m.flight_time_s + lead_s;  // the LLO plane holds the site then
        const time::Epoch start = c.touchdown - *m.touchdown_after_epoch_s;
        // a session of its own, as the flight's: the prototype's sources, grids and fidelity flags
        FlightSession session(*sources_.ephemeris, *sources_.earth, config_dir_, sources_.gravity, sources_.moon,
                              sources_.ephemeris_node_spacing_s > 0.0);
        session.set_moon_surface_detail(prototype_.moon_surface_detail());
        session.set_moon_rocks(prototype_.moon_rocks());
        session.set_boil_off(true);  // as start_mission
        const auto started = start_lunar_landing_mission(session, config_dir_, m, start, nullptr, &site_);
        if (!started.refusal.ok()) return {std::nullopt, started.refusal};
        if (!started.lunar_planned_touchdown || !started.lunar_budget_margin_kg) {
            return {std::nullopt, report::make(report::Code::internal_error)};
        }
        const bool nrho = m.lunar_orbit == "nrho";
        time::Epoch touchdown = *started.lunar_planned_touchdown;
        double inclination_rad = mission::LloTarget{}.inclination_rad;  // the NRHO's LLO: polar
        if (!nrho) {
            const auto descent = descent_plan(session, m, start);
            if (!descent.feasible) return {std::nullopt, descent.reason};
            touchdown = descent.site_pass_epoch;
            if (std::getenv("STARSIM_DEBUG_LUNAR_WINDOWS")) {
                const auto* p = session.lunar_outbound_plan();
                const math::Vec3 site_dir =
                    session.moon_orientation()->fixed_to_gcrf(c.touchdown).rotation *
                    math::Vec3{std::cos(site_.latitude_rad) * std::cos(site_.longitude_rad),
                               std::cos(site_.latitude_rad) * std::sin(site_.longitude_rad), std::sin(site_.latitude_rad)};
                std::fprintf(stderr,
                             "[lunar_windows] plan td %+.3f h, plane-site %.2f km at the candidate, arrival %+.2f h, "
                             "pass %+.3f d, crossrange %.1f km\n",
                             p->llo_touchdown ? (*p->llo_touchdown - c.touchdown) / 3600.0 : -999.0,
                             p->llo_plane_normal_moon_inertial
                                 ? constants::moon_mean_radius_m *
                                       std::asin(std::abs(dot(*p->llo_plane_normal_moon_inertial, site_dir))) / 1000.0
                                 : -1.0,
                             (p->insertion.epoch - start) / 3600.0 - m.flight_time_s / 3600.0,
                             (touchdown - c.touchdown) / 86400.0, descent.crossrange_before_trim_m / 1000.0);
            }
            const auto* plan = session.lunar_outbound_plan();
            if (plan->llo_plane_normal_moon_inertial) {
                const math::Vec3 pole = session.moon_orientation()->fixed_to_gcrf(touchdown).rotation *
                                        math::Vec3{0.0, 0.0, 1.0};
                inclination_rad =
                    std::acos(std::clamp(dot(*plan->llo_plane_normal_moon_inertial, pole), -1.0, 1.0));
            } else if (m.llo_inclination_rad) {
                inclination_rad = *m.llo_inclination_rad;
            }
        }
        // the hard Sun rule on the touchdown the flight takes: daylight from it through the stay and the ascent
        const auto& day = c.daylight;
        if (touchdown < day.sunrise || touchdown + m.surface_stay_s + lunar_window_ascent_lead_s > day.sunset) {
            if (std::getenv("STARSIM_DEBUG_LUNAR_WINDOWS")) {
                std::fprintf(stderr, "[lunar_windows] touchdown %+.3f d from the candidate, daylight %.3f .. %.3f d\n",
                             (touchdown - c.touchdown) / 86400.0, (day.sunrise - c.touchdown) / 86400.0,
                             (day.sunset - c.touchdown) / 86400.0);
            }
            return {std::nullopt, no_daylight(m.surface_stay_s, day.sunset - std::max(touchdown, day.sunrise))};
        }
        const auto sun = mission::site_sun(*sources_.ephemeris, *sources_.moon, site_.latitude_rad,
                                           site_.longitude_rad, touchdown);
        LunarLandingWindow w;
        w.mission_start = start;
        w.touchdown = touchdown;
        w.liftoff = touchdown + m.surface_stay_s;
        w.sun_elevation_rad = sun.elevation_rad;
        w.sun_azimuth_rad = sun.azimuth_rad;
        w.daylight_after_touchdown_s = day.sunset - touchdown;
        w.propellant_reserve_kg = *started.lunar_budget_margin_kg;
        w.llo_inclination_rad = inclination_rad;
        w.orbit = nrho ? mission::LunarOrbitKind::Nrho : mission::LunarOrbitKind::Llo;
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

    const FlightSession& prototype_;
    const std::filesystem::path& config_dir_;
    const config::LunarLandingMission& m_;
    const config::LunarSiteConfig& site_;
    const vehicle::EnvironmentSources& sources_;
    double settling_s_;
};

LunarWindowResult search(const FlightSession& prototype, const std::filesystem::path& config_dir,
                         const LunarWindowRequest& request, const std::function<bool(double)>& progress,
                         bool& cancelled) {
    const auto report_progress = [&](double p) {
        if (progress && !progress(std::clamp(p, 0.0, 1.0))) cancelled = true;
        return !cancelled;
    };
    LunarWindowResult result;
    const auto cancel = [&] {
        result.refusal = report::make(report::Code::lunar_window_search_cancelled);
        return result;
    };
    if (!report_progress(0.0)) return cancel();
    const auto& sources = prototype.environment_sources();
    if (!sources.moon) {
        result.refusal = no_window(report::make(report::Code::lunar_landing_no_moon_orientation));
        return result;
    }
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
    double longest_s = 0.0;
    std::vector<Candidate> candidates;
    for (const auto& day : days) {
        longest_s = std::max(longest_s, day.sunset - day.sunrise);
        for (time::Epoch t = day.sunrise + 0.5 * lunar_window_candidate_step_s;
             t <= last && t + hold_s <= day.sunset; t += lunar_window_candidate_step_s) {
            if (t < earliest) continue;
            const auto sun = mission::site_sun(*sources.ephemeris, *sources.moon, lat, lon, t);
            candidates.push_back({t, day, band_distance(sun.elevation_rad, windows)});
        }
    }
    if (candidates.empty()) {
        result.refusal = no_window(no_daylight(m.surface_stay_s, longest_s));
        report_progress(1.0);
        return result;
    }
    // 3: by preference, ties chronological
    std::stable_sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
        return a.band_distance_rad < b.band_distance_rad;
    });
    if (!report_progress(scan_progress)) return cancel();

    // the site's surface held for the whole search (the sessions share it); its tile missing refuses every candidate
    std::shared_ptr<const environment::DetailedLunarTerrain> surface;
    try {
        surface = shared_moon_surface(config_dir, config_dir.parent_path() / "data", {true, true}, request.site);
    } catch (const std::exception& e) {
        result.refusal = no_window(detail::exception_report(e));
        report_progress(1.0);
        return result;
    }

    // 4: the real chain, best first, until `count` pass. Spread over the lunar days first (a list of dates, not five
    // slots of one morning): a candidate whose daylight already holds a window waits for a second round.
    const Chain chain(prototype, config_dir, m, request.site, lunar_llo_settling_s(m, gm_moon));
    CauseTally causes;
    const size_t wanted = static_cast<size_t>(windows.count);
    const bool debug = std::getenv("STARSIM_DEBUG_LUNAR_WINDOWS") != nullptr;
    std::vector<time::Epoch> days_with_window;  // by sunrise
    std::vector<size_t> deferred;
    size_t tried = 0;
    const auto began = std::chrono::steady_clock::now();
    const auto attempt = [&](size_t i) {
        const auto outcome = chain.run(candidates[i], lead_s);
        if (outcome.window) {
            result.windows.push_back(*outcome.window);
            days_with_window.push_back(candidates[i].daylight.sunrise);
        } else {
            causes.add(candidate_cause(outcome.cause));
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
    report_progress(1.0);
    return result;
}

}  // namespace

LunarWindowResult find_lunar_landing_windows(const FlightSession& prototype, const std::filesystem::path& config_dir,
                                             const LunarWindowRequest& request,
                                             const std::function<bool(double)>& progress) {
    const auto key = cache_key(prototype, config_dir, request);
    {
        const std::lock_guard lock(cache_mutex());
        const auto it = cache().find(key);
        if (it != cache().end()) {
            if (progress) progress(1.0);
            return it->second;
        }
    }
    bool cancelled = false;
    auto result = search(prototype, config_dir, request, progress, cancelled);
    if (!cancelled) {
        const std::lock_guard lock(cache_mutex());
        cache().emplace(key, result);
    }
    return result;
}

}  // namespace starsim::sim
