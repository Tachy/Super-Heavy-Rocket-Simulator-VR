#pragma once

#include <filesystem>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "starsim/config/lunar_site_config.hpp"
#include "starsim/mission/lunar_outbound.hpp"
#include "starsim/report/report.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/time/epoch.hpp"

namespace starsim::sim {

// Where to land, how long to stay, from when on (lunar sites plan Task 8). `mission_id`: a lunar landing mission of
// config/missions (its vehicle, orbit kind, budget and [windows]); its own site and stay are replaced by these.
struct LunarWindowRequest {
    std::string mission_id;
    config::LunarSiteConfig site;
    double stay_s = 0.0;
    time::Epoch start;  // the earliest mission start
};

// A landing date that flies: the mission started at `mission_start` from the parking orbit is planned for this
// touchdown by the same planners the flight uses (outbound plan, touchdown point, budget, descent plan from the
// predicted LLO). To fly it, start the mission at `mission_start` with [descent] touchdown_after_epoch =
// touchdown - mission_start (the LLO plane chosen for the listed touchdown; the search chose it for the candidate,
// within ~1 h of it - test "every listed Apollo 15 window flies"). NRHO windows: `touchdown` is the NRHO departure's
// planned pass (no descent prediction) and `llo_inclination_rad` the polar default of the LLO below the NRHO.
struct LunarLandingWindow {
    time::Epoch mission_start, touchdown, liftoff;   // liftoff = touchdown + stay
    double sun_elevation_rad = 0.0;                  // at touchdown, above the site's geometric horizon
    double sun_azimuth_rad = 0.0;                    // from north, east positive
    double daylight_after_touchdown_s = 0.0;         // to sunset (or the end of the scanned span at polar sites)
    // the budget's margin above the mission's reserve after every leg incl. the turn back onto the landing plane the
    // stay causes (lunar_stay_plane_change_mps, ruling R15)
    double propellant_reserve_kg = 0.0;
    double llo_inclination_rad = 0.0;                // the LLO plane against the MOON_ME equator at touchdown
    mission::LunarOrbitKind orbit = mission::LunarOrbitKind::Llo;
};

// `windows`: by preference (Sun elevation's distance to the [windows] band, ties chronological). `refusal`: set when
// `windows` is empty - lunar.no_window(cause) with the most frequent cause across the candidates
// (lunar.no_daylight_for_stay, lunar.budget_short, lunar.no_site_pass, lunar.nrho_site_unreachable, or what else
// refused them, e.g. a missing site tile) - or lunar.window_search_cancelled (`windows`: those found until then).
struct LunarWindowResult {
    std::vector<LunarLandingWindow> windows;
    std::optional<report::Report> refusal;
};

// What the search takes from the flying session, as values: its sources (the raw ephemeris / orientations are not
// owned - whoever owns them, e.g. FlightSim, keeps them alive until the search returns; the shared data is immutable)
// and its Moon surface fidelity flags.
struct LunarWindowEnvironment {
    vehicle::EnvironmentSources sources;
    bool moon_surface_detail = true;
    bool moon_rocks = true;
};
// The snapshot of `session`: take it on the session's thread (the sim thread writes the flags).
LunarWindowEnvironment lunar_window_environment(const FlightSession& session);

// The landing windows of `request` within the mission's [windows] horizon_days after request.start:
// 1. daylight intervals at the site (mission::daylight_intervals) over the horizon and the stay after it;
// 2. touchdown candidates every lunar_window_candidate_step_s inside intervals that still hold the stay and
//    lunar_window_ascent_lead_s after it, and no earlier than the transfer and the descent lead after the start;
// 3. ranked by the Sun elevation's distance to [windows] sun_preferred_min/max_deg, ties chronological;
// 4. in that order the real chain per candidate - mission start = touchdown - transfer - descent lead
//    (start_lunar_landing_mission on a session of its own: placement, outbound plan with the LLO plane through the
//    site at that touchdown, touchdown point, budget incl. the stay's plane change), then the descent plan from the
//    predicted LLO (TLI and insertion flown impulsively, planned once settled in the LLO as the session does). The
//    plane-through-site transfer lets the arrival drift from start + transfer; when the descent misses the planned
//    pass for it, the start is re-solved earlier by the drift (plane epoch kept, never before request.start, up to
//    lunar_window_max_resolves times); a pass still missed is lunar.arrival_after_site_pass unless the later pass
//    still holds the stay in daylight. The touchdown must lie in the daylight holding the stay - until [windows]
//    count pass. The list spreads over the lunar days first: a candidate whose daylight already holds a window is
//    tried only after every other candidate (dates, not one morning's slots).
// NRHO missions (ruling R16): before the candidates the budget is bounded from below once - the outbound at the
// mission's [windows] nrho_outbound_floor_mps (the least the planner flies from the parking orbit, measured), the legs
// after the arrival nominal, the site-dependent legs (nrho_plane_change, stay_plane_change) at 0; when even that does
// not close, every candidate is refused at once with lunar.nrho_site_unreachable.
// Runs on the caller's thread (seconds to minutes); `progress` gets 0..1 and cancels the search when it returns false.
// Results are cached in memory per (config dir, mission, site, stay, start to the hour, flags) - only complete searches
// whose refusals do not come from missing data or the environment (a later download takes effect); a cached list
// whose mission start lies before request.start (same hour) is searched again.
LunarWindowResult find_lunar_landing_windows(const LunarWindowEnvironment& environment,
                                             const std::filesystem::path& config_dir, const LunarWindowRequest& request,
                                             const std::function<bool(double)>& progress);
// The same with lunar_window_environment(prototype), taken at the call (prototype is not read afterwards: call it on
// prototype's thread, or snapshot there and pass the environment).
LunarWindowResult find_lunar_landing_windows(const FlightSession& prototype, const std::filesystem::path& config_dir,
                                             const LunarWindowRequest& request,
                                             const std::function<bool(double)>& progress);

// Empties the result cache (tests; data under the config dir replaced).
void clear_lunar_window_cache();

// [E] re-solves of the mission start per candidate for the arrival drift (fixed point; 1 sufficed where measured).
inline constexpr int lunar_window_max_resolves = 3;

// [E] spacing of the touchdown candidates (brief: every 6 h; the Sun climbs ~3 deg in 6 h at the equator).
inline constexpr double lunar_window_candidate_step_s = 6.0 * 3600.0;
// [E] daylight kept after the stay: the liftoff, the ascent to insertion (~7 min) and the touchdown's slip from the
// listed pass (braking phase ~10 min) stay in sunlight.
inline constexpr double lunar_window_ascent_lead_s = 3600.0;

}  // namespace starsim::sim
